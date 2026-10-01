# PushReverseStepper - known issues

Status update: 2026-10-01

### Fixed
1. ~~**Holding the start button leaves the car with no control loop.**~~
   **FIXED (2026-10-01):** Removed `while (digitalRead(BUTTON_PIN) == LOW);`. `phase` transitions immediately to `FORWARD`, ensuring heading correction, encoder differential trim, and the distance deceleration ramp run every pass without delay.

### Open Problems
2. **Turning WL_SW off after boot doesn't actually disable wireless.**
   It only clears `bleConnected` and the LED. `CmdCallbacks::onWrite` still
   passes every write to `handleCommand` (START/STOP/setters all work), and the
   next client connect sets `bleConnected` back to true in `onConnect`.

3. **BLE commands run concurrently with `loop()`.**
   `onWrite` calls `handleCommand` from the BLE task, so it races the main loop:
   e.g. `CAL_START` zeroes `totalTicks` mid-`updateEncoder`, and START/STOP can
   change `phase` partway through a loop pass. Fix: queue the command string and
   handle it inside `loop()`.

4. **`STOP` while idle is reported as a failed run.**
   It goes through `finishRun`, setting `phase = DONE` and turning the LED red.
   Cosmetic.

### New Features & Tuning Notes (2026-10-01)
- **Closed-Loop Encoder Differential Trim:** Added `rt_KP_ENC` (default 0.20 st/s per tick diff, clamped to ±150 st/s). Balances modest tick differences (~100–1000 ticks) by nudging wheel speeds. When the gyro is actively correcting heading error (`fabs(headingError) > 0.20°`), opposing encoder trim is attenuated by 90% so it doesn't fight turns. Configurable live via BLE (`KP_ENC:<val>` / `KE:<val>`), persistent in NVS (`kp_enc`), and reflected in the dashboard UI.
- **Start-Line Gyro Re-Zeroing:** When `startRunSequence()` is called, the stationary gyro bias is refreshed via 40 fast samples on the line right before motor motion. This prevents warm-up/thermal drift from causing steady heading bias.
- **Heading & Cross-Track Gains:** `rt_KP_HEADING` raised from 12.0 to 28.0 and `rt_KX` raised from 20.0 to 50.0 (defaults). When an initial launch twitch creates lateral translation, the car actively angles back onto the start axis rather than driving parallel-shifted.

