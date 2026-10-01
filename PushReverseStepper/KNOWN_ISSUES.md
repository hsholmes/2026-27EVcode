# PushReverseStepper - known issues

Open problems found in code review (2026-09-30). None fixed yet.

1. **Holding the start button leaves the car with no control loop.**
   `loop()` busy-waits in `while (digitalRead(BUTTON_PIN) == LOW);` right after
   `startRunSequence()`. The step timer keeps driving the motors, but heading
   correction, the encoder distance stop and the decel ramp are frozen until the
   button is released - only the step cap can stop the car.

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
