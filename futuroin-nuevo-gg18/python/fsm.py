"""fsm.py — SHARED autonomous race controller for both WRO FE stages.

The MCU owns the fast PD steering loop; this is the high-level state machine on the
Linux side. It consumes VisionCommands (one per camera frame, from vision.py) and
drives the car through the bridge_io RPC wrappers.

Shared flow (both stages):
    WAIT  -> START (set_straight)
    DRIVE -> 3 laps; count corners from the orange/blue line events (vision latches one
             fire per physical line; we add a time cooldown as belt-and-suspenders)
    Stage 1 (open):     after corner #12 -> FINISH  (finish the sweep, then drive back
                        ONTO THE START POSITION and stop — matched by front-ranger distance)
    Stage 2 (obstacle): after corner #12 -> PARK    (run the injected parking routine)

Pillar passing (Stage 2) is produced in vision.py (decide_obstacle) and arrives as
cmd.steer_bias, so this controller stays almost stage-agnostic — the only Stage-2
branch is the PARK hand-off.

NOTE: every threshold here is a STARTING POINT to tune on the real mat (see the brief
in other/fabrication-challenges.md and the README).
"""
import threading
import time

import bridge_io as io

TOTAL_CORNERS = 12        # 3 laps x 4 corners. WRO: a lap counts on EXIT of corner #12.
FINISH_COAST_S = 1.6      # FALLBACK stop only: seconds past the 12th sweep when the ranger
                          # gave no start-position fingerprint (sensor absent / RPC dead)

# --- return-to-start finish (Stage 1) ----------------------------------------------
# WRO rule: after 3 laps the car must stop in the SECTION IT STARTED FROM — the ask is
# to stop ON the start position. The front ultrasonic gives that almost for free:
# standing on the start position, the distance to the wall ahead is a position
# fingerprint, and after 12 corners (3 full laps, same heading) the wall ahead on the
# final straight is the SAME wall. start() samples the fingerprint before the car
# moves; after the 12th sweep completes we creep down the start section and stop when
# the ranger reads that distance again.
FINISH_APPROACH_PCT = 45   # % speed cap on the final straight (precision beats pace)
FINISH_BRAKE_MM     = 150  # command the stop this much BEFORE the fingerprint distance:
                           # braking/coast distance eats it. Nose stops short of the mark
                           # -> lower it; rolls past the mark -> raise it. TUNE ON MAT.
FINISH_POLL_S       = 0.10 # fast ranger poll on the final approach (MCU refresh ~60 ms)
FINISH_CONFIRM      = 2    # consecutive under-threshold readings to accept (glitch filter)
FINISH_SETTLE_S     = 0.4  # arm the distance check only this long after the sweep reaches
                           # target: mid-wiggle the cone can glance a side wall and return
                           # a short FALSE reading that must not stop the run early
FINISH_MAX_S        = 4.0  # hard cap on the final approach: stop even without a match
                           # (ranger died mid-run) — better a stop near the section than
                           # driving on into the far corner zone
CORNER_COOLDOWN_S = 2.5   # ignore new corner events this long after a turn (anti double-count;
                          # must outlast the corner sweep + exit-line crossing, but stay well
                          # under a straight-section transit so real corners never get eaten)

# --- front-halt deadlock breaker ("un-stick") -------------------------------------
# vision halts the car when a wall fills the front view; but once stopped the view
# never changes, so without recovery the car would stand there for the rest of the
# round. After UNSTICK_AFTER_S halted we back up briefly (the MCU PD loop stays
# stable in reverse — the sketch flips the correction sign with drive dir), then
# resume and let vision re-decide. Same idea as the old Pi Main.py "wall close ->
# reverse + look for a new approach".
UNSTICK_AFTER_S   = 2.0   # s of continuous halt in DRIVE before backing up
UNSTICK_REVERSE_S = 0.7   # s of reverse pulse
UNSTICK_SPEED     = 45    # % drive speed during the pulse

# --- front ultrasonic (HC-SR04, MCU get_front_mm RPC) -----------------------------
# THE fix for the reported "turned at the corner, then never went forward" bug:
# vision read the front as CLEAR while the nose was against the wall, so speed
# stayed at 70% and the old halt-detector (speed==0) never fired. The ranger sees
# the wall regardless of image framing: the MCU blocks forward drive itself, and
# HERE we detect the ranger-standstill and run the same reverse un-stick pulse.
FRONT_STUCK_MM = 110      # ranger closer than this while DRIVing = pressed on a wall
                          # (halved from 220 -- user: react 50% closer to the wall)
FRONT_CLEAR_MM = 150      # reverse pulse may end early once clearance exceeds this
FRONT_POLL_S   = 0.25     # how often to poll the MCU for the front distance

# --- side ultrasonics: SMALL-range collision guard --------------------------------
# The LEFT/RIGHT HC-SR04s are a NEAR-COLLISION guard ONLY (user: "small range, not
# fully"): inside SIDE_AVOID_MM the car gets a fixed steer bias AWAY from that wall;
# beyond it the side readings are ignored -- no full-range lane centering, the
# CAMERA keeps steering priority. The guard is also OFF mid-sweep: while the gyro
# is bringing the car around a 90-deg corner it legitimately swings close to the
# inner wall, and a bias there would bend the arc the gyro is checking.
SIDE_POLL_S     = 0.10    # side-ranger poll period (MCU refreshes each side ~60 ms)
SIDE_AVOID_MM   = 100     # react only when a side wall is CLOSER than this (mm).
                          # TUNE ON MAT: scraping walls -> raise toward 150;
                          # zig-zagging in the 600 mm corridor -> lower toward 80.
SIDE_AVOID_BIAS = 10      # deg of steer away from the too-close wall (small, so the
                          # camera's own bias stays the dominant signal)
MAX_TOTAL_BIAS  = 28      # clamp on vision bias + side guard (matches vision.py)

# While a 90-deg sweep is in progress the car has to keep rolling to come around
# (Ackermann steering cannot rotate in place), so this speed is FORCED for the
# whole sweep -- it overrides whatever vision commands, including a halt. A real
# collision is still guarded by the front ultrasonic (the MCU blocks forward
# under FRONT_HALT_MM itself).
TURN_DRIVE_PCT = 60       # % motor power through a corner (user: raised 35 -> 60)


class RaceController:
    """stage = 'open' | 'obstacle'. on_park = callable(direction:int) for Stage 2 only."""

    def __init__(self, stage="open", on_park=None):
        self.stage = stage
        self.on_park = on_park
        self.state = "WAIT"
        self.corner_count = 0
        self.turn_direction = 0          # latched +90 (CW/right) or -90 (CCW/left)
        self._last_corner_t = -1e9       # NOT 0.0: with a clock that starts near 0 (sim,
                                         # monotonic time) 0.0 means "corner at t=0" and the
                                         # cooldown silently eats a corner in the first second
        self._finish_t = None
        self._start_front_mm = None      # front distance ON the start position (fingerprint)
        self._sweep_done_t = None        # when the 12th corner sweep reached its target
        self._finish_hits = 0            # consecutive under-threshold readings (glitch filter)
        self._last_speed = None
        self._last_bias = None
        self._halt_t = None              # when the current continuous halt started
        self._unstick_t = 0.0            # when the reverse pulse started
        self._front_mm = None            # last front-ultrasonic reading (throttled poll)
        self._front_t = 0.0              # when we last polled it
        self._left_mm = None             # last LEFT side-ranger reading (throttled poll)
        self._right_mm = None            # last RIGHT side-ranger reading
        self._side_t = 0.0               # when the sides were last polled
        self._last_tel = 0.0             # last telemetry print

    def start(self):
        # Fingerprint the start position BEFORE moving: median front distance while
        # the car still stands on it. The 3-lap FINISH drives back to this reading.
        self._start_front_mm = self._sample_front()
        io.set_straight()                # zero heading, motor + hold ON
        self.state = "DRIVE"
        print(f"[fsm] START — stage={self.stage} "
              f"front@start={self._start_front_mm}mm", flush=True)

    @staticmethod
    def _sample_front(n=7, gap_s=0.05):
        """Median of n front-ranger reads (~0.35 s, car standing still). None when
        nothing valid came back (sensor absent / wall beyond range) -> FINISH then
        falls back to the timed coast instead of the distance match."""
        vals = []
        for _ in range(n):
            f = io.get_front_mm()
            if isinstance(f, int) and f > 0:
                vals.append(f)
            time.sleep(gap_s)
        if not vals:
            return None
        vals.sort()
        return vals[len(vals) // 2]

    # ---- called by VisionPipeline once per camera frame ----
    def on_vision(self, cmd):
        if self.state == "FINISH":
            self._service_finish(cmd)
            return
        if self.state in ("PARK", "STOPPED"):
            return
        if self.state == "UNSTICK":
            self._service_unstick()
            return
        if self.state != "DRIVE":
            return

        now = time.time()

        # is a 90-deg sweep still in progress? (None = RPC hiccup -> treat as reached)
        at_ok = io.at_target()
        turning = at_ok is False

        # forward proximity speed + lateral bias (centering / pillar passing) to the MCU
        # (speed_pct None = "leave speed unchanged", e.g. the camera-off command).
        # Mid-turn the sweep speed is FORCED to TURN_DRIVE_PCT (see above): the car
        # drives THROUGH the corner at full turn power instead of freezing or
        # coasting on whatever vision happened to command.
        speed = cmd.speed_pct
        if turning and speed is not None:
            speed = TURN_DRIVE_PCT
        if speed is not None and speed != self._last_speed:
            io.set_speed(speed)
            self._last_speed = speed

        # side-ranger near-collision guard (throttled poll; see SIDE_* above).
        # Added ON TOP of vision's bias, but only outside a sweep -- mid-turn the
        # car intentionally passes close to the inner wall and the pure gyro arc
        # must not be bent by a panic bias.
        if now - self._side_t >= SIDE_POLL_S:
            self._side_t = now
            self._left_mm = io.get_left_mm()
            self._right_mm = io.get_right_mm()
        bias = cmd.steer_bias + (0 if turning else self._side_avoid_bias())
        bias = max(-MAX_TOTAL_BIAS, min(MAX_TOTAL_BIAS, bias))
        if bias != self._last_bias:
            io.nudge(bias)
            self._last_bias = bias

        # throttled telemetry -> watch the corner behaviour in the App Lab console
        if now - self._last_tel > 0.7:
            self._last_tel = now
            print(f"[tel] {self.state} corner={self.corner_count}/{TOTAL_CORNERS} "
                  f"head={io.get_heading()} tgt={io.get_target()} "
                  f"spd={speed} bias={bias} turn={cmd.turn} "
                  f"L={self._left_mm} R={self._right_mm} | {cmd.note}",
                  flush=True)

        # front-ultrasonic poll (throttled). Catches the reported bug: vision says
        # CLEAR while the nose is against a wall the camera band missed. The MCU
        # blocks forward on the ranger by itself; here we NOTICE the standstill.
        if now - self._front_t >= FRONT_POLL_S:
            self._front_t = now
            self._front_mm = io.get_front_mm()
        f = self._front_mm
        wall_pressed = isinstance(f, int) and 0 <= f < FRONT_STUCK_MM

        # deadlock breaker: vision-halt OR ranger-wall persisting -> reverse pulse.
        # Uses the POST-override speed: a camera halt that was floored to
        # TURN_DRIVE_PCT mid-turn is not "stuck" (the car is rolling) -- but the
        # ranger pressed-on-wall signal still counts even while turning.
        if speed == 0 or wall_pressed:
            if self._halt_t is None:
                self._halt_t = now
            elif now - self._halt_t > UNSTICK_AFTER_S:
                why = f"ultrasonic {f}mm" if wall_pressed else "vision halt"
                print(f"[fsm] stuck ({why}) -> UNSTICK (reverse pulse)", flush=True)
                io.set_drive_dir(-1)
                io.set_speed(UNSTICK_SPEED)
                self._unstick_t = now
                self.state = "UNSTICK"
                return
        else:
            self._halt_t = None

        # corner event — vision already latched it to fire once per physical line.
        # at_target gate: every WRO corner zone has BOTH an orange and a blue line
        # (entry + exit). Without this gate the EXIT line re-fires turn() while the
        # car is still sweeping the corner -> corners double-count -> "12 corners"
        # after 6 and a premature stop. While the 90-deg sweep is in progress
        # at_target() is False, so exit-line events are swallowed. (None = RPC
        # hiccup -> fail-open so a dropped call can never freeze corner counting.
        # at_ok was polled once above, before the speed override.)
        if (cmd.turn != 0 and (now - self._last_corner_t) > CORNER_COOLDOWN_S
                and at_ok is not False):
            self._last_corner_t = now
            self.turn_direction = cmd.turn
            io.turn(cmd.turn)
            self.corner_count += 1
            print(f"[fsm] corner {self.corner_count}/{TOTAL_CORNERS} -> turn({cmd.turn})",
                  flush=True)
            if self.corner_count >= TOTAL_CORNERS:
                self._begin_finish()

    def _side_avoid_bias(self):
        """Small-range side guard: a fixed bias AWAY from a side wall closer than
        SIDE_AVOID_MM, 0 otherwise. -1/None (open corridor / no sensor / RPC fail)
        never triggers. Both sides close (tight corridor) -> away from the CLOSER
        one only, so the guard can't oscillate the car between two near walls."""
        l, r = self._left_mm, self._right_mm
        l_close = isinstance(l, int) and 0 <= l < SIDE_AVOID_MM
        r_close = isinstance(r, int) and 0 <= r < SIDE_AVOID_MM
        if l_close and r_close:
            return SIDE_AVOID_BIAS if l < r else -SIDE_AVOID_BIAS
        if l_close:
            return SIDE_AVOID_BIAS       # wall close on the LEFT -> steer right
        if r_close:
            return -SIDE_AVOID_BIAS      # wall close on the RIGHT -> steer left
        return 0

    def _begin_finish(self):
        # the lap counts on EXIT of corner #12 -> coast out first, THEN stop / park
        if self.stage == "obstacle" and self.on_park:
            self.state = "PARK"
            print("[fsm] 12 corners -> PARK", flush=True)
            threading.Thread(target=self._do_park, daemon=True).start()
        else:
            self.state = "FINISH"
            self._finish_t = time.time()
            self._sweep_done_t = None
            self._finish_hits = 0
            print("[fsm] 12 corners -> FINISH (sweep the corner, then home in on "
                  f"front@start={self._start_front_mm}mm)", flush=True)

    def _service_finish(self, cmd):
        """FINISH = the return-to-start stop, serviced once per camera frame.
        Phase 1 — corner #12's sweep is still in progress: keep TURN_DRIVE_PCT,
        exactly like any other corner. Phase 2 — sweep reached its target: creep
        down the start section at FINISH_APPROACH_PCT and stop when the front
        ranger reads the start-position fingerprint again (or on the FINISH_MAX_S
        cap). Lane-centering bias stays live throughout so the final straight is
        driven centered, not frozen on the last mid-corner bias."""
        now = time.time()

        if cmd.steer_bias != self._last_bias:
            io.nudge(cmd.steer_bias)
            self._last_bias = cmd.steer_bias

        sweeping = io.at_target() is False        # None (RPC hiccup) = treat as done
        if sweeping:
            self._sweep_done_t = None             # still coming around the corner
            speed = TURN_DRIVE_PCT if cmd.speed_pct is not None else None
        else:
            if self._sweep_done_t is None:
                self._sweep_done_t = now
                print("[fsm] finish: corner #12 swept -> final approach", flush=True)
            speed = cmd.speed_pct
            if speed is not None:
                speed = min(speed, FINISH_APPROACH_PCT)
        if speed is not None and speed != self._last_speed:
            io.set_speed(speed)
            self._last_speed = speed

        if self._sweep_done_t is None:
            return
        since_sweep = now - self._sweep_done_t

        d0 = self._start_front_mm
        if d0 is None:
            # no fingerprint (ranger absent at start) -> the old timed stop, but
            # correctly referenced to SWEEP DONE instead of the entry-line event
            if since_sweep >= FINISH_COAST_S:
                self._stop_run(f"timed {FINISH_COAST_S}s coast — no start fingerprint")
            return

        if since_sweep < FINISH_SETTLE_S:         # let the steering settle straight
            return
        if now - self._front_t >= FINISH_POLL_S:
            self._front_t = now
            self._front_mm = io.get_front_mm()
            f = self._front_mm
            threshold = d0 + FINISH_BRAKE_MM
            if isinstance(f, int) and 0 < f <= threshold:
                self._finish_hits += 1
            else:
                self._finish_hits = 0
            if now - self._last_tel > 0.7:
                self._last_tel = now
                print(f"[tel] FINISH front={f}mm stop@<={threshold}mm "
                      f"hits={self._finish_hits}/{FINISH_CONFIRM}", flush=True)
            if self._finish_hits >= FINISH_CONFIRM:
                self._stop_run(f"front {f}mm matched start {d0}mm "
                               f"(+{FINISH_BRAKE_MM}mm brake margin)")
                return
        if since_sweep >= FINISH_MAX_S:
            self._stop_run(f"final-approach cap {FINISH_MAX_S}s — front={self._front_mm}mm")

    def _stop_run(self, why):
        io.stop()
        self.state = "STOPPED"
        print(f"[fsm] STOPPED — 3 laps complete ({why})", flush=True)

    def _service_unstick(self):
        """End the reverse pulse: forward again, hand speed control back to vision.
        Ends EARLY once the front ultrasonic reports real clearance (no over-backing)."""
        done = time.time() - self._unstick_t >= UNSTICK_REVERSE_S
        if not done:
            f = io.get_front_mm()
            if isinstance(f, int) and f > FRONT_CLEAR_MM:   # -1 (no sensor) stays timed
                done = True
        if done:
            io.set_drive_dir(+1)
            io.set_speed(0)              # stay stopped for one frame...
            self._last_speed = None      # ...so vision's next command re-sends speed
            self._halt_t = None
            self._front_mm = None        # stale "pressed" reading must not re-trigger
            self.state = "DRIVE"
            print("[fsm] unstick done -> DRIVE", flush=True)

    def _do_park(self):
        try:
            self.on_park(self.turn_direction or 90)
        finally:
            io.stop()
            self.state = "STOPPED"
            print("[fsm] STOPPED — parked", flush=True)
