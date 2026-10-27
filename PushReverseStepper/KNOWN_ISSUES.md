# PushReverseStepper - known issues

Status update: 2026-10-27 (re-checked against the current code: `PushReverseStepper.ino` + `BLE.ino`)

No open problems. The 2026-10-27 fixes compile but have not been tested on the car yet - see "Still to test" below.

### Fixed
1. ~~**Holding the start button leaves the car with no control loop.**~~
   **FIXED (2026-10-01):** Removed `while (digitalRead(BUTTON_PIN) == LOW);`. `phase` transitions immediately to `FORWARD`, ensuring heading correction, encoder differential trim, and the distance deceleration ramp run every pass without delay.
   Re-checked 2026-10-27: still fixed.

2. ~~**Turning WL_SW off after boot doesn't actually disable wireless.**~~
   **FIXED (2026-10-27):** Switching WL_SW off now stops advertising, disconnects any connected client and clears queued commands (`bleCheckSwitch()` in `BLE.ino`). While it's off, `onWrite` ignores commands, `onConnect` refuses new connections, and `onDisconnect` doesn't restart advertising. Switching it back on restarts advertising without a reset. The radio itself stays powered once it has been started - reset the board for a run where it must be fully off.

3. ~~**BLE commands run concurrently with `loop()`.**~~
   **FIXED (2026-10-27):** `onWrite` now copies the command into a FreeRTOS queue (16 commands, up to 95 characters each). `loop()` runs them through `bleProcessCommands()`, right after Serial commands, so commands never change state partway through a loop pass. The config readback on connect also moved into `loop()` (`configPending`, sent by `bleService()`), so the BLE task no longer sends anything itself. If the queue is full, the command is dropped and `[BLE] Command queue full - dropped` is printed. The dashboard's "send all" waits between writes, so this shouldn't happen.

4. ~~**`STOP` while idle is reported as a failed run.**~~
   **FIXED (2026-10-27):** `STOP` only calls `finishRun()` during `FORWARD`, `PAUSE` or `REVERSE`. When idle or done it replies `STATUS:Not running - nothing to stop` and changes nothing: no red LED, no DONE phase, no stray log sample.

5. ~~**The DONE log sample recorded the wrong distance.**~~
   **FIXED (2026-10-27):** `finishRun()`'s final sample logged the left encoder's whole-run total against the forward target, even after a reverse leg. It now logs the last leg's distance and target, matching the DONE telemetry. All three log-sample copies became one `logRunSample()`.

6. ~~**Two BLE sends skipped the `statusChar` null check.**~~
   **FIXED (2026-10-27):** "Running reverse!" and "Forward done, pausing" checked only `bleConnected`. Every send now goes through `bleSend()` in `BLE.ino`, which checks both.

7. ~~**The route simulator ramps up slightly differently from `loop()`.**~~
   **FIXED (2026-10-27):** `simulateLegS()` now ramps from `MIN_SAFE_SPEED + accel * t`, the same as `loop()` does at the start of a leg. Its leg-time estimates are now slightly shorter, so total-time mode and the bottle-catch speed choose a slightly slower cruise. `MT` (fastest possible total) on the dashboard also drops slightly. The plain TIME_F/TIME_R speed formula (no bottle, total-time off) is unchanged. **Re-check timed runs on the car.**

8. ~~**Gyro calibration with no good reads sets the offset to NaN.**~~
   **FIXED (2026-10-27):** `mpuCalibrateGyro()` now returns `false` when no read succeeds (printing `Calibration FAILED: no gyro reads succeeded`). `setup()` then sets `hasImu = false` and prints `[4] MPU-6050 FAILED - continuing without heading correction`, the same as when the IMU isn't found.

### Still to test (on the car)
- Total-time mode and the bottle catch: check the run time still lands on target after fix 7.
- WL_SW: flip it off while connected (the dashboard should disconnect and stop seeing the car), then on again (it should reconnect).
- Dashboard "send all" settings and START/STOP still work through the command queue.

### Before a competition run
- `FORWARD_ONLY` is still `true` (calibration mode: the run stops after the forward leg). Set it to `false` for the full push-reverse run.

### New Features & Tuning Notes (2026-10-01)
- **Closed-Loop Encoder Differential Trim:** Added `rt_KP_ENC` (default 0.20 st/s per tick diff, clamped to ±150 st/s). Balances modest tick differences (~100–1000 ticks) by nudging wheel speeds. When the gyro is actively correcting heading error (`fabs(headingError) > 0.20°`), opposing encoder trim is attenuated by 90% so it doesn't fight turns. Configurable live via BLE (`KP_ENC:<val>` / `KE:<val>`), persistent in NVS (`kp_enc`), and reflected in the dashboard UI.
- **Start-Line Gyro Re-Zeroing:** When `startRunSequence()` is called, the stationary gyro bias is refreshed via 40 fast samples on the line right before motor motion. This prevents warm-up/thermal drift from causing steady heading bias.
- **Heading & Cross-Track Gains:** `rt_KP_HEADING` raised from 12.0 to 28.0 and `rt_KX` raised from 20.0 to 50.0 (defaults). When an initial launch twitch creates lateral translation, the car actively angles back onto the start axis rather than driving parallel-shifted.

### Code Cleanup (2026-10-27)
Same behaviour, less code (1,893 → 1,790 lines before the fixes above; 1,250 excluding comments and blanks):
- Bluetooth moved into its own tab, `BLE.ino`. The main file only calls its functions (`bleSend()`, `bleSendAlert()`, `sendVehicleConfig()`, `bleService()`, `bleProcessCommands()`, `bleSetup()`, `bleCheckSwitch()`).
- Simple `KEY:value` settings are one `SETTINGS[]` table, used by both `loadPreferences()` and `handleCommand()`. NVS keys are unchanged.
- `Serial.print` chains became `Serial.printf`. Debug numbers that land exactly halfway (e.g. 284.25) may round to the other digit; BLE text is unchanged.
- Shared helpers: `beginLeg()`, `solveCruise()`, `toMeters()`, `zeroOdometry()`, `clearLiveTerms()`, `signedTicks()`, `formatLogLine()`.
