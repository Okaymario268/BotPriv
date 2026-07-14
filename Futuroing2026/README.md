# Futuroing2026 — Servo on the Arduino UNO Q (App Lab)

This is an **Arduino App Lab** app for the **Arduino UNO Q**. It fixes the
"`Servo.h` not detected" error you hit when moving the robot code from the
classic Arduino IDE, and runs a servo program on the UNO Q's MCU.

Your original full-robot sketch is kept in this folder as `Futuroing2026.ino`
(the Stage 2 reference). The App Lab app uses `sketch/sketch.ino`.

## Why the IDE error happened (and the fix)

Arduino App Lab does **not** read the classic Arduino IDE library folder
(`...\Documentos\Arduino\libraries\`). App Lab resolves each project's C++/MCU
libraries from **`sketch/sketch.yaml`** instead. Because `Servo` was never
declared there, App Lab couldn't find `Servo.h` — even though the IDE had it.

The fix lives in [`sketch/sketch.yaml`](sketch/sketch.yaml), which must use App Lab's **profiles**
format. A plain `default_fqbn` + `libraries` list triggers
`Missing Profile name` on `arduino:zephyr > 0.54.1`. Leave `fqbn:` blank — App Lab fills it from the
connected UNO Q:

```yaml
profiles:
  default:
    fqbn:
    platforms:
      - platform: arduino:zephyr
    libraries:
      - Servo (1.3.0)

default_profile: default
```

`Servo 1.3.0` is the version that added Zephyr / UNO Q support (its
`architectures` list includes `zephyr`). Older versions abort with
`#error "This library only supports boards with an AVR, SAM, SAMD, NRF52, STM32F4, Renesas or XMC processor."`.

## Project layout

```
Futuroing2026/
├── app.yaml              # App Lab app manifest (best-effort; App Lab may regenerate it)
├── Futuroing2026.ino     # your ORIGINAL full-robot sketch (Stage 2 reference, untouched)
├── python/
│   ├── main.py           # keep-alive Python brick (servo logic is in the sketch)
│   └── requirements.txt
└── sketch/
    ├── sketch.ino        # STAGE 1: minimal servo sweep/center test (active)
    └── sketch.yaml       # declares Servo 1.3.0 + board  <-- the actual fix
```

## Opening this project in App Lab

1. Open **Arduino App Lab** → **Open App** → choose this `Futuroing2026` folder.
   - **If App Lab won't open this hand-made folder:** create a **New App** in App Lab,
     then copy this folder's `sketch/` and `python/` folders into the new app (App Lab
     will write its own correct `app.yaml`).
2. Open the **sketch (C++)** editor. Confirm **Servo** shows up as a dependency.
   If it doesn't, click **Add Library**, search **Servo**, and add it — that writes
   `Servo (1.3.0)` into `sketch.yaml` in App Lab's exact format.
3. Connect the UNO Q, select it, and click **Run**. App Lab compiles + uploads the
   sketch to the STM32 MCU and starts the Python brick.

## Verify

- **Compile:** build succeeds with no `Servo.h: No such file or directory`.
- **Hardware:** servo + external 5–6 V servo supply (share ground with the board) →
  the servo sweeps left/right/center on a loop.

## UNO Q / Zephyr gotchas

- **PWM pins:** on Zephyr both `servo.attach(pin)` and `analogWrite(pin, …)` need a pin
  wired to a hardware PWM channel in the UNO Q device tree — not every Uno digital pin
  qualifies. If the servo doesn't move on **D11** (or the motor PWM on **D3**), move to
  another PWM header pin and re-test. This is the most common follow-up after the include is fixed.
- Keep **Servo ≥ 1.3.0** in `sketch.yaml`.
- If **Add Library** can't find Servo, update App Lab and the `arduino:zephyr` core.
- `Serial` works but is routed through App Lab's bridge/monitor, not the IDE Serial Monitor.

---

## STAGE 2 — full FuturoIng robot port

Once Stage 1 passes, replace the contents of [`sketch/sketch.ino`](sketch/sketch.ino)
with the port below (the same code as your original `Futuroing2026.ino`: servo steering +
L298N DC motor). Only the library-resolution path changed — the Servo API is identical.

```cpp
#include <Servo.h>

// Pins for Motor (L298N)
int MotorVelocityPin = 3;   // ENA / PWM speed  -> must be PWM-capable on the UNO Q
int MotorPin1        = 6;   // IN1 direction
int MotorPin2        = 5;   // IN2 direction

// Motor values
int CurrentVelocity = 150;

// Servo angles
int CenterServoRotation = 138;
int LeftServoRotation   = 125;
int RightServoRotation  = 150;

// Ultrasonic max ranges (cm) — used by your sensor logic
int FrontDistanceRange = 35;
int LeftDistanceRange  = 50;
int RightDistanceRange = 50;

// Servo
int ServoPin = 11;          // verify PWM-capable on the UNO Q

// Debug / mode flags
bool Debug = false;
bool ButtonEnabled = true;
bool EnableObstacleChallenge = true;
int  MaxYellowToDetect = 12;

Servo servo;

void MotorBackward() {
  digitalWrite(MotorPin1, LOW);
  digitalWrite(MotorPin2, HIGH);
}

void MotorForward() {
  digitalWrite(MotorPin1, HIGH);
  digitalWrite(MotorPin2, LOW);
}

void MotorStop() {
  digitalWrite(MotorPin1, LOW);
  digitalWrite(MotorPin2, LOW);
}

void ServoRotation(int Angle) {
  servo.write(Angle);
}

void setup() {
  // Init motor
  pinMode(MotorPin1, OUTPUT);
  pinMode(MotorPin2, OUTPUT);
  pinMode(MotorVelocityPin, OUTPUT);
  analogWrite(MotorVelocityPin, CurrentVelocity);

  // Init serial + servo
  Serial.begin(9600);
  servo.attach(ServoPin);
  servo.write(96);   // center on startup
}

void loop() {
  // TODO: your obstacle-avoidance / steering logic.
  // Example helpers are ready:
  //   ServoRotation(CenterServoRotation);
  //   MotorForward();
}
```
