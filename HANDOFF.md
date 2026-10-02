# Handoff — Scioly EV 2026-27 (state as of commit `3026093` + uncommitted ramp-up fix, 2026-09-30)

Read this first. It's the context a fresh agent needs to pick up where the
last session stopped.

## Ground rules from the user

- **Only edit files in this repo** (`EV202627`, remote `hsholmes/2026-27EVcode`).
  The older `C:\Users\Holmes\Desktop\Coding\SciolyEV` folder is retired.
- The user is a student building the car; they test on real hardware and
  report back with dashboard screenshots or log lines. Nothing here has
  been compiled by an agent — the user compiles and flashes in Arduino IDE.
- Ask before installing third-party skills/packages (the `find-skills`
  skill, installed globally, tells the agent it may auto-install — don't).

## The task

Push-reverse event: drive straight forward a set distance (pushing a
bottle past a line), pause, reverse a set distance, stop at the Target
Point. Autonomous once started. NiMH AA pack only, no lithium, no
capacitor banks, no remote control during a run.

## Files

| File | What it is |
|---|---|
| `PushReverseStepper/PushReverseStepper.ino` | **The active firmware.** ESP32-S3. |
| [`PushReverseStepper/KNOWN_ISSUES.md`](PushReverseStepper/KNOWN_ISSUES.md) | Open firmware bugs from the 2026-09-30 code review (button busy-wait, WL_SW not disabling BLE, BLE/loop race, idle STOP). None fixed yet. |
| `SciolyEV_Dashboard_v3.html` | **The active dashboard.** Web Bluetooth, open in Chrome/Edge. |
| `CurveRouteStepper/CurveRouteStepper.ino` | Sibling firmware written by someone else for the same dashboard. Not in use, but it has features ours doesn't (see "Not ported"). |

A long reference doc, `SciOlyEV_Overview.md`, lives in the old SciolyEV
folder (hardware table, full pin history, protocol). It's accurate as of
the cross-track change but doesn't cover the tracking telemetry or the
removed time limit. Don't edit it there; ask if it should be copied in.

## Hardware facts (all confirmed on the real car — don't re-derive from the schematic)

The schematic transcription was wrong for many pins; these were fixed by
live testing:

| Signal | GPIO | Notes |
|---|---|---|
| STEP_A / DIR_A / EN_A (left) | 5 / 4 / 6 | EN active-low |
| STEP_B / DIR_B / EN_B (right) | 9 / 46 / 14 | EN active-low |
| START button (SW5) | 7 | INPUT_PULLUP, LOW = pressed |
| WL_SW wireless switch (SW4) | 15 | LOW = wireless on |
| WL_LED | 16 | firmware-driven |
| Status LED (D5) | 40 | low-voltage flasher |
| VOLT rail sense | 13 | 100k/10k divider (×11), read with `analogReadMilliVolts` |
| RGB LED (on DevKitC) | 48 | inferred, not schematic-confirmed |
| I2C bus 0 (`Wire`) SDA/SCL | 1 / 2 | MPU-6050 + **right** AS5600 |
| I2C bus 1 (`Wire1`) SDA/SCL | 41 / 42 | **left** AS5600 |

- SW6 "Reset" is wired to the module's EN pin — a real reboot, not a GPIO.
- IMU is an **MPU-6050** (clone; WHO_AM_I reads 0x74, accepted alongside 0x68). Not an ICM-20948.
- **Gyro sign: positive heading = nose turned left (CCW from above).** Confirmed by hand.
- `setDirection()`: forward = LEFT_DIR HIGH, RIGHT_DIR LOW. This was flipped on 9/29 after the car drove backward and circled (backward motion turned the heading correction into positive feedback).
- Wheels 76.2 mm, 200 steps/rev × 8 microstep. Encoder calibration on the car: ~16475 ticks/m (vs 17113 theoretical).
- The car was dropped on ~9/17; afterwards the 12 V rail collapsed under load. VM–GND measured 1500 Ω (not a short). The user later ran it again, so power seems resolved, but root cause wasn't identified.

## Firmware — how it works now

- State machine: `WAIT_BUTTON → FORWARD → PAUSE → REVERSE → DONE`. Start via button or BLE `START`.
- **Step engine (2026-09-30):** one hardware GPTimer per wheel (1 MHz, created in `setup()` so the ISR runs on core 1, away from BLE on core 0) fires each STEP pulse and re-arms for exactly one step interval. The ISR is integer-only (no float in ISRs on the S3). A 1 kHz `esp_timer` task slews each wheel's speed toward its target and converts it to an interval. This replaced a 20 kHz `esp_timer` tick-counter on core 0, which had 200/250 µs step-gap jitter at 4500 st/s and was delayed by BLE. Compile-checked with arduino-cli; **not yet run on the car**.
- **Stall recovery (2026-09-30):** every 100 ms each wheel's encoder travel is compared with its steps; under 50% → the leg's ramp restarts from 30% of the stall speed, the stalled wheel's speed snaps down, and the rest of the run is capped at 85% of the stall speed. Sends `STATUS:STALL <L|R> at N st/s - top speed now M st/s`; the leg summary adds `| stalls N`. Not yet triggered on the car.
- Trapezoidal profile (ramp-up over `ACCEL_TIME_SEC`, **distance-based ramp-down** from encoder remaining distance). Speed = steps / (time − ACCEL_TIME_SEC), clamped 80–4500 st/s.
- **Both ramps are computed in `loop()`** (see "Ramp-up fix" below). Ramp-up = `accelRate × (millis() − startTime)`, clamped between `MIN_SAFE_SPEED` and cruise; ramp-down = `sqrt(2·a·remaining)`, and the smaller of the two wins so a short leg never jumps up to meet the ramp-down curve. `startLeg(stepCap, accel)` starts both wheels at `MIN_SAFE_SPEED`; the ISR's per-wheel slew runs at `WHEEL_SLEW_FACTOR` (4×) the leg accel and only smooths between loop passes.
- **Stop = encoder distance reached.** Only fallback is the step cap (1.3× nominal steps). **The time-limit stop was removed on request** — Time F/R now only set cruise speed.
- Steering: P-only on heading, **cross-track correction** (sideways offset integrated from encoder travel × sin(heading); target heading = −dirSign × KX × offset, capped ±5°), plus **closed-loop encoder differential trim (2026-10-01)**: balances small tick errors (~100–1000 ticks) by nudging wheel speeds by `rt_KP_ENC` (default 0.2 st/s per tick difference, clamped to ±150 st/s). When the gyro is actively steering to correct heading error, opposing encoder trim is attenuated by 90% so it never fights turns. Heading and elapsed time freeze in DONE.
- Motors stay armed in DONE (holding torque). Params persist in NVS (`Preferences`, namespace `scioly_ev`).
- Compile gotchas already handled (keep them if refactoring): explicit prototypes after `struct Encoder` / `enum RunPhase` (Arduino auto-prototype bug); BLE uses Arduino `String`, not `std::string`, on core 3.3.x; `min/max` need plain-float snapshots of volatiles. Needs the Adafruit NeoPixel library.

## BLE protocol (matches the dashboard)

Commands: `START`, `STOP`, `RESET`, `ARM`, `DISARM`, `GET_CONFIG`,
`CAL_START`, `CAL_FINISH:<m>`, `CAL:<ticks/m>`, `CALIBRATE:<measured m>`,
and setters `DIST_F/DIST_R` (m), `TIME_F/TIME_R` (s), `KP`, `KX`, `KP_ENC` (or `KE`),
`BIAS_F/BIAS_R` (raw st/s, + steers right), `PAUSE` (ms),
`DIST_FACTOR:<x>` (0.5–1.5) and `DIST_MEASURED:<m>` (factor ×= DIST_F ÷ measured).

**Bottle catch (2026-10-01):** `BOTTLE:<m>` (bottle distance from the start, car length subtracted like DIST_F; 0 = off, NVS `bottle_m`) and `CATCH_V:<m/s>` (default 0.30, NVS `catch_mps`); CFG keys `BP`/`CV`. Forward leg only: slows at the leg accel to the catch speed from 10 cm before the bottle to 10 cm past it, then re-accelerates at half the accel (`catchSpeedLimit()`). `recalcRoute()` simulates the leg (`simulateForwardLegS`) and bisects for the cruise speed that still meets TIME_F; the catch costs ~1.3–1.7 s, so tight times hit the 4500 st/s cap. Dashboard: Bottle Position (with 2/3/4 m presets) and Catch Speed in Run Target Parameters, included in Sync All. Compile-checked, not yet run on the car.

**Max speed (2026-10-02):** the old `MAX_SAFE_SPEED` const is now `rt_MAX_SPEED` (st/s, default 5000 ≈ 0.75 m/s, NVS `max_sps`), set with `MAX_V:<m/s>` (0.20–1.50). Leg speeds, the total-time split, the stall cap and the "fastest possible" `MT` all use it; CFG adds `MV` (m/s). Dashboard "Max Speed" field next to Catch Speed, in Sync All before TOTAL_TIME.

**Car length (2026-10-02):** DIST_F/DIST_R are entered as competition distances; each leg travels that minus `CAR_LENGTH_M` (0.207 m), floor 0.05 m, via `fwdTravelM()`/`revTravelM()`. Used for the stop distance, step caps, speeds, simulations, bottle check, `CALIBRATE`/`DIST_MEASURED` (measured value = how far the car moved) and telemetry `T:`. CFG still echoes the entered distances. Bottle position is entered the same way: the catch uses `bottleTravelM()` = BOTTLE − car length (catch off if that's ≤ 0). Dashboard mirrors the constant (`CAR_LENGTH_M` in the HTML) for the offline Target Distance display.

**Total run time (2026-10-02):** `TOTAL_TIME:<s>` (0 = off, NVS `total_s`). When on, `recalcRoute()` → `applyTotalTime()` bisects one cruise speed for both legs so simulated forward leg (incl. bottle catch) + pause + reverse leg = total (forward leg only in FORWARD_ONLY), and writes the resulting TIME_F/TIME_R. Re-solves whenever distances, bottle, catch speed, pause or factor change. A manual `TIME_F`/`TIME_R` switches it off. CFG adds `TT` (total) and `MT` (fastest possible total at the 4500 st/s cap); TF/TR now sent with 2 decimals. Dashboard: "Total Run Time" field first in Run Target Parameters; while on, the leg-time boxes are read-only and show the car's computed times, and Sync All skips them and sends TOTAL_TIME last. Compile-checked; solver checked against a Python copy of the simulation; not yet run on the car.

**Encoder trim (2026-10-01):** `rt_KP_ENC` (default 0.2 st/s per tick diff, NVS key `kp_enc`). Set via `KP_ENC:<val>` or `KE:<val>`. Dashboard has a KE field in the parameter grid.

**Distance factor (2026-09-30):** separate from ticks/m. Each leg stops at
set distance × factor of encoder distance; step caps/speeds use the scaled
distance too. Telemetry `D:` is encoder distance ÷ factor and `T:` the set
distance, so the dashboard shows real-world distance. NVS key `dist_factor`.
Dashboard has its own "Distance Factor" card (Factor + Measured Run Distance).

Telemetry (10 Hz, all phases):
`P: D: T: V: H: G: B: L: R: CL: CR: M: E: WL: WR: X:` — CL/CR = live motor
speed multipliers vs ramped cruise speed, WL/WR = per-wheel tracking %
(encoder rotation ÷ commanded steps), X = cross-track mm (+ = left).
Config readback: `CFG:DF DR TF TR KP BF BR P C KX DK KE` (KE = encoder trim gain).
At each leg end the car sends `STATUS:<FWD|REV> track min L:x% R:y% | Δticks ±N | offset ±Nmm | stalls N`.

## Dashboard changes made

- KE (Encoder Trim) input field added to the parameter grid, with live NVS sync and Send All inclusion.
- Live motor compensation `CL:` and `CR:` telemetry parsed and displayed in the "Live Motor Compensation" card.
- Drift label and compass needle flipped to match the gyro sign.
- KX field added; bias fields relabeled "st/s (+ steers R)".
- **Upload bug fixed:** every setter makes the car echo its whole config, which used to overwrite not-yet-sent fields, so Sync All / Reset & Arm only really sent the first value. Edited fields are now tracked as dirty and protected until sent. Verified with a mock car in the browser.
- New "Wheel Tracking" row (L%/R%, laggier wheel goes amber) and cross-track offset badge.

## Ramp-up fix (applied 2026-09-30, uncommitted, not yet tested on the car)

Found while looking at the veer-at-start issue below. Previously `startLeg()`
set both wheels' target straight to cruise speed and the timer ISR slewed
each wheel up at the leg accel. During the whole ramp-up both wheels were
accel-limited and climbed in lockstep, so the heading/cross-track correction
`loop()` wrote into the per-wheel targets had **no effect until the ramp
ended** — exactly when the car was veering.

Changes in `PushReverseStepper.ino`:
- New constant `WHEEL_SLEW_FACTOR = 4.0`; `activeAccel = accel × 4`.
- `startLeg(stepCap, accel)` — the `speed` argument is gone; targets start at `MIN_SAFE_SPEED`. Both call sites (forward start, reverse start) updated.
- `loop()` computes the time-based ramp-up itself (uses `millis()`, not the cached `now`, because a BLE `START` can set `startTime` after `now` was sampled and the subtraction would underflow), then applies `min()` with the distance-based ramp-down, then adds the per-wheel steering correction as before.

Net effect: steering correction now acts from the first step of each leg.
The user should check whether the start-of-leg veer shrinks. If it's gone,
the "right wheel lag" hypothesis below may have been this software bug; if
it persists, continue with the hardware checks.

## Finding 2026-09-30: the drift is traction / weight balance

- Old-firmware run at 7 m / 10 s (user pressed STOP at 3 s): tracking L 100% / R 100% (no lost steps), yet the right wheel turned ~14% more than the left and the car still yawed 21° right → the right tire was slipping on the floor.
- The battery sat on the left. **The user moved it to the right and the car now drifts left.** The unloaded wheel slips. Fix is centering the weight between the drive wheels, not the motor.
- The left encoder counts negative going forward (mirrored motors). Control uses per-leg magnitudes so it never mattered; `LEFT_ENC_INVERT = true` now makes telemetry L/R both positive forward, which also fixes the dashboard's "Δ ticks" badge (it was L − R with opposite signs).

**Update:** with the battery centered the car drives straight.

## Current mode: FORWARD_ONLY (2026-09-30)

`const bool FORWARD_ONLY = true` in the firmware: the run ends after the forward leg (green LED, `STOP:Forward complete (forward-only mode)`), for calibrating distance with the dashboard's Measured Distance → `CALIBRATE:<m>`. **Set it back to false** for the full push-reverse run.

## Open issue — where to pick up

**Lateral translation / offset to the left after launch**:
The user observed that encoder corrections successfully balance the wheels (within 20 ticks), and the vehicle heading angle straightens out parallel to the track, but the car ends up translated (shifted) slightly to the left of the start axis on every run.

**Root causes & fixes (2026-10-01):**
1. **Launch angle kick & slow heading response**: When accelerating from a stop, any slight initial angle error causes the car to travel diagonally for the first second. With `rt_KP_HEADING = 12.0`, heading correction was too sluggish. Boosted default to `rt_KP_HEADING = 28.0` so heading snaps straight within centimeters.
2. **Weak cross-track recovery**: With `rt_KX = 20.0`, a 30 mm lateral shift commanded only a 0.6° target heading back to the line, requiring over 10 seconds of travel to eliminate. Boosted default to `rt_KX = 50.0` so the car actively crabs back onto the original center line.
3. **Start-line gyro thermal drift**: The MPU-6050 was only calibrated once in `setup()` at cold boot; sitting on the line for 1–2 minutes introduced a 0.2–0.5 dps thermal bias. Added an automatic 40-sample stationary gyro re-zero in `startRunSequence()` right before the motors step.
4. **Steering bias defaults**: Changed firmware and dashboard defaults for `rt_FORWARD_BIAS` and `rt_REVERSE_BIAS` from `10.0` to `0.0`.

User action items:
- Flash latest firmware.
- In dashboard, verify `KP Heading = 28.0`, `KX Cross-Track = 50.0`, `Forward Bias = 0.0`, and `KE = 0.20`. Click **SEND ALL** or **RESET & ARM**.
- Observe if the car now tracks down the center line and recovers from any initial launch twitch.

## Not ported from `CurveRouteStepper.ino` (deliberately)

PD gyro control (KD term), dynamic pacing speed-boost, stepper-coil audio tones.
(Note: Closed-loop encoder differential trim was ported on 2026-10-01 per user request).
