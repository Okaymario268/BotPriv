# gg18 — two-line corner confirmation, side-ultrasonic guard, steering trim

Base: `FuturoIn NUEVO gg17` (the zip). Every change below is tunable from the
"BOT CONFIGURATION" blocks at the top of each file — nothing else needs editing
on the mat.

## 1. The bug being fixed

The bot followed the corridor and, the moment it detected the first corner-line
colour (orange, driving clockwise), it fired the full 90° turn immediately.
That starts the arc at the **outer edge** of the corner zone, so the car comes
around early and drives into the **inner wall**.

## 2. Two-line corner sequence (`python/vision.py`)

Every WRO corner zone has BOTH colours: driving **clockwise** the car meets
**orange first, then blue**; driving **counter-clockwise** it meets **blue
first, then orange**. The corner trigger is now a two-phase sequence:

1. **Entry line REGISTERS** — when the run's entry colour sits low in the frame
   (the existing `LINE_FIRE_ROW` gate, debounced by `CORNER_STREAK`), the
   pipeline only *registers* the corner (`phase: watch → confirm`). No turn yet.
2. **Drive a little forward** — the car keeps driving into the zone for at
   least `CONFIRM_MIN_S` (0.25 s), which puts it **between the two lines** even
   when both are visible in the same frame.
3. **Confirmation line FIRES the turn** — once the *other* colour shows
   ≥ `CONFIRM_MIN_PIX` pixels (and the minimum forward time has passed),
   `turn(±90)` fires. The arc now starts from between the lines instead of at
   the first line.
4. **Timeout fallback** — if the confirmation colour is never seen
   (`CONFIRM_TIMEOUT_S` = 2.0 s: missed line, occlusion, bad HSV), the turn
   fires anyway so one missed line can never strand the run.

Direction authority stays with the **camera**: the first colour seen in the run
latches CW (+90, orange-first) or CCW (−90, blue-first), and the confirmation
colour is always the opposite one. Because only the entry colour can *register*
and the exit colour is *consumed* as the confirmation, the old exit-line
double-count problem disappears structurally; the FSM's `CORNER_COOLDOWN_S` and
`at_target()` gates stay in place as belt-and-suspenders.

The **gyroscope still owns the rotation**: `turn(±90)` only bumps the MCU's
target heading, the 50 Hz PD loop sweeps the car around while integrating the
gyro, and `at_target()` (±`TARGET_TOL` = 5°) reports when the measured rotation
has actually reached 90°. The FSM keeps using that signal to gate corner
counting and force `TURN_DRIVE_PCT` through the sweep.

Console tags to watch while tuning:

    [vision] entry line registered (...) -> driving between the lines
    [vision] confirmation line seen (...) -> turn(±90)
    [vision] NO confirmation line after 2.00s -> turn(±90) on timeout

Tuning: still clipping the inner wall → raise `CONFIRM_MIN_S`; turning too late
or overshooting the zone → lower it. Frequent timeout fires → the confirmation
colour's HSV band or `CONFIRM_MIN_PIX` needs work (check the annotated feed).

## 3. Side ultrasonics — small-range collision guard

New hardware support: two more HC-SR04s, LEFT and RIGHT, mid-body, aimed square
to the car's axis.

* **`sketch/sketch.ino`** — pins `LEFT TRIG=D5 / ECHO=D6`, `RIGHT TRIG=A2 /
  ECHO=A3` (every ECHO through the same 1 kΩ/2 kΩ divider — 5 V sensor, 3.3 V
  GPIO). **Never A4/A5**: that is the gyro's I²C bus; an HC-SR04 echo pin would
  clamp SCL and kill the gyro (and with it every turn). Sampling is **polled**
  (no extra `attachInterrupt` — EXTI-collision risk on the Zephyr core),
  alternates sides every `SIDE_PING_MS` = 30 ms (no cross-talk), and runs a
  median-of-3 per side. New RPCs: `get_left_mm` / `get_right_mm`
  (−1 = open corridor / no echo / sensor absent).
* **`python/bridge_io.py`** — `get_left_mm()` / `get_right_mm()` wrappers.
* **`python/fsm.py`** — the guard itself, deliberately **small range, not
  full-range**: only when a side wall is closer than `SIDE_AVOID_MM` = 100 mm
  does the FSM add a fixed `SIDE_AVOID_BIAS` = 10° steer *away* from it, on top
  of the camera's bias (clamped to ±`MAX_TOTAL_BIAS` = 28°). Beyond 100 mm the
  side readings are ignored — no lane centering, the camera keeps priority.
  The guard is **disabled mid-sweep**: while the gyro is bringing the car
  around a 90° corner it legitimately passes close to the inner wall, and a
  panic bias there would bend the pure gyro arc. If both sides are close at
  once, it steers away from the closer one only (no zig-zag).
* **`python/main.py`** — `/api/heading` now also returns `left` / `right` so
  the bench UI can watch the side rangers live.

Telemetry: the `[tel]` heartbeat now prints `L=<mm> R=<mm>`.

## 4. Mechanical right-pull compensation (`sketch/sketch.ino`)

The car has a slight built-in pull to the **right** (steering-linkage bug).
With the servo at `CENTER` it veers right, and the PD loop can only cancel a
standing pull by settling at a heading error of ~pull/`KP`. New constant:

    #define STEER_TRIM  -2   // deg added to every auto-steer write

The trim (negative = a touch of left steer) is applied inside the PD write and
at every auto recenter (`set_straight`, `set_steer` off, `stop`), so the loop
works around a true-straight center. Manual `set_angle()` stays untrimmed (raw
bench control). Tune on the mat: still drifts right → more negative; drifts
left → back toward 0.

## 5. Instant start — button press starts the run, no countdown

The START button used to play a 3-2-1 LED-matrix countdown (~3 s) before
releasing the run. New firmware switch:

    #define USE_COUNTDOWN  0   // 0 = GO the instant the button is pressed
                               // 1 = old 3-2-1 countdown behaviour

With `USE_COUNTDOWN 0` (the new default) the debounced press jumps the start
gate straight to FIRED: the console logs
`start button pressed -> GO (countdown disabled)` and the Python side's
`start_ready()` poll releases immediately. Nothing else changes — the button
gate itself (`USE_START_BUTTON`), `arm_start`, `set_start_button` and
`start_ready` all behave exactly as before, just without the 3-second wait.

## Files touched

| File | Change |
|---|---|
| `sketch/sketch.ino` | side ultrasonics (pins, polled sampler, median-of-3, `get_left_mm`/`get_right_mm` RPCs), `STEER_TRIM` |
| `python/vision.py` | two-line corner sequence (`CONFIRM_MIN_PIX/MIN_S/TIMEOUT_S`, watch→confirm phases) |
| `python/fsm.py` | small-range side guard (`SIDE_POLL_S/AVOID_MM/AVOID_BIAS`), off mid-sweep, `L=/R=` telemetry |
| `python/bridge_io.py` | `get_left_mm()` / `get_right_mm()` |
| `python/main.py` | `/api/heading` returns `left`/`right` |
| `DOCUMENTATION.txt` | header note pointing here |

## Bench checklist before a run

1. Wire the two side sensors (dividers!) and power up: the console must print
   `HC-SR04 side ultrasonics: LEFT TRIG=D5/ECHO=D6, RIGHT TRIG=A2/ECHO=A3 ...`
   and the gyro's `MPU WHO_AM_I = 0x68 (ok)` must STILL appear (proves the
   sensors didn't land on A4/A5).
2. `/api/heading` on the bench UI: hand-wave at each side sensor and watch
   `left`/`right` move; > ~2.5 m or no sensor reads −1.
3. Roll the car by hand over a taped orange+blue pair: console shows
   `entry line registered` on the first colour and `-> turn(...)` only at the
   second.
4. Drive straight on the gyro hold: if it still pulls right, lower `STEER_TRIM`
   (more negative) by 1 and re-test.
