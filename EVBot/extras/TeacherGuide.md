# EVBot Teacher Guide

EVBot is an Arduino library for teaching grades 4–7 to code in C++ with a real robot car. Students write short programs (`robot.forward(30);`, `robot.turnLeft(90);`) and the car does the rest. It drives straight, turns accurately, and explains mistakes in plain English.

The car is the Science Olympiad Electric Vehicle (ESP32-S3, two stepper motors, wheel encoders and a gyro). Everything that keeps it accurate runs automatically in the background, so students never see it.

---

## 1. Setting up (once per computer)

1. Install **Arduino IDE 2.x** from arduino.cc.
2. Add the ESP32 boards. In **File → Preferences → Additional boards manager URLs** add:
   `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
   Then in **Tools → Board → Boards Manager**, search **esp32** and install **esp32 by Espressif Systems**, version **3.x** (tested with 3.3.12).
3. Install EVBot. Either:
   - **Sketch → Include Library → Add .ZIP Library…** and choose a zip of the `EVBot` folder, or
   - copy the `EVBot` folder into `Documents/Arduino/libraries/`.

   Then restart the IDE.
4. Plug the car in with USB. Choose **Tools → Board → esp32 → ESP32S3 Dev Module** and the car's port under **Tools → Port**.
5. Open **File → Examples → EVBot → 01_HelloLight** and click **Upload**. Open **Tools → Serial Monitor** at **115200 baud**. You should see `[EVBot] Ready!` and `Hello! I am a robot.`

No other libraries are needed.

### Wireless upload over Bluetooth (optional)

Students can send programs to the car without a cable, using the **EVBot Uploader** page (`extras/EVBotUploader.html`). Code is still written and compiled in Arduino IDE on the laptop. Nothing goes online.

**Once per car (teacher, over USB):**
1. Open **File → Examples → EVBot → Teacher_CarSetup**, set `CAR_NAME` (for example `EVBot-03`) and `CAR_PIN` (4–8 digits), and upload it with the USB cable.
2. Put a sticker on the car with its name and PIN.
3. Upload any lesson once over USB. Every program that calls `robot.begin()` keeps wireless upload working from then on.

**Once per laptop:** copy `EVBotUploader.html` to the desktop. It works offline. Open it in **Chrome or Edge** (Safari and Firefox can't use Bluetooth). Arduino IDE must use the **Default** partition scheme (**Tools → Partition Scheme**) for wireless updates to have room.

**Each upload (student):**
1. Turn the car's **wireless switch (WL_SW)** on. The wireless LED lights when Bluetooth is ready, about 7 seconds after power-on.
2. In Arduino IDE: **Sketch → Export Compiled Binary**.
3. In the uploader page: **Connect** → pick the car → type the PIN (only the first time) → **Choose file** (the `.ino.bin` file in the sketch's `build/esp32.esp32.esp32s3/` folder; only the first time) → **Upload**.
4. The car restarts with the new program and the page reconnects. `robot.say()` messages appear in the page's message box.

Notes:
- After changing code, students must **Export again** before clicking Upload. The page warns if the file hasn't changed since the last upload.
- **Stop robot** stops the wheels immediately; **Run program again** restarts the program.
- If a program crashes before `robot.begin()`, or wireless upload stops working, upload once over USB to recover.
- With the switch off, the car has no Bluetooth at all.

## 2. Before each class

- **Charge the batteries.** `robot.battery()` reads about 12 V when full. The small status LED flashes when the battery is below 11 V.
- **Check the car:** upload **SquareTest** (a teacher example) and press START. The car should drive a 40 cm square left, then right, and finish on its start spot.
- **Floor space:** about 1.5 m × 1.5 m per car, on a smooth hard floor. Painter's tape is useful for start lines and targets.
- **Cables:** keep the USB cable slack, or unplug it after uploading. A cable that pulls on the car makes it swerve.

## 3. Safety and handling rules for students

1. **Hands off at startup.** After `robot.begin()` the light is amber for about 7 seconds while the gyro calibrates. It turns blue when ready.
2. **Floor only.** Never run the car on a table.
3. **Hands away from the wheels** while the light is green (moving).
4. **Press START, then step back.** The car waits for the button (`robot.waitForButton()`) so nobody is holding it when it moves.
5. **Switch the car off** when changing batteries or wiring.

## 4. The lessons

Every lesson is an example sketch (**File → Examples → EVBot**). Each one teaches one idea, runs as-is, and ends with a "Try this!" list that gets harder as it goes. Plan about **30–45 minutes per lesson**: 5 minutes to introduce, 5 to run the example, and the rest for the challenges.

| # | Lesson | Big idea | New commands / C++ | Students can… |
|---|---|---|---|---|
| 1 | HelloLight | A program is a list of steps; `setup()` vs `loop()` | `begin`, `light`, `wait`, `say` | upload a program, make a traffic light |
| 2 | FirstDrive | Commands take numbers (parameters); units | `forward`, `backward`, `waitForButton`, `setSpeed` | stop the car on a tape mark |
| 3 | Turns | Sequence; degrees | `turnLeft`, `turnRight` | drive a square and a triangle |
| 4 | Repeat | Loops save writing the same code | `for` | draw polygons with 2 lines of code |
| 5 | Variables | Named boxes for numbers; math | `int`, `float`, `/` | draw any polygon; draw a spiral |
| 6 | MyFunctions | Make your own commands | `void name(int x) { }` | build shapes from shapes |
| 7 | ButtonIf | Decisions | `if`, `else if`, `else`, `==`, `buttonPressed` | make a button-controlled dance |
| 8 | Sensors | The robot measures the world | `distance`, `heading`, `battery` | predict, then check measurements |
| 9 | WhileLoop | Repeat until something happens | `while`, `&&`, `!` | stop the car with the button |
| 10 | Challenge | Put it all together | `millis()` | run a mini Science Olympiad event |

### Lesson notes

**1 · HelloLight.** The car does not move, so it can stay on the desk. Point out that the lines run in order, top to bottom. Show the Serial Monitor; many students have never seen one. *Ask:* "What would happen if we put the lights in `loop()` instead?" (They would repeat forever.)

**2 · FirstDrive.** First time on the floor, so go over the safety rules. Try #4 shows a friendly error (`forward(-10)`). Use it to show that the robot explains mistakes. *Ask:* "Why does the robot wait for the button?"

**3 · Turns.** Students write 8 lines for a square. Let them feel how repetitive that is, because that motivates Lesson 4. The triangle challenge (120°) is a good moment to discuss exterior angles. *Common mistake:* turning 60° for a triangle (the inside angle).

**4 · Repeat.** Walk through the three parts of the `for` loop on the board. Counting from 0 is surprising; have students print `i` with `robot.say(i);`.

**5 · Variables.** The magic is `360.0 / sides`. Try #5 shows integer division (`360 / 7` is 51, not 51.43). For grade 4, skip it or demonstrate it.

**6 · MyFunctions.** Compare a function to a recipe card: written once, used many times. Parameters change the result ("a recipe for *N* cookies").

**7 · ButtonIf.** The program is in `loop()`, which keeps checking the button. Watch for `=` (assign) vs `==` (compare); the compiler accepts `if (presses = 1)` but it is always true.

**8 · Sensors.** Have students write down their predictions **before** running it. Discuss why `distance()` goes down when reversing and why turning doesn't change it. `heading()` doesn't wrap around: four right turns read 360, not 0.

**9 · WhileLoop.** Contrast with `for`: you don't know how many hops it will take. The button is only checked between hops, so students must **hold** it. Try #5 (infinite loop) is worth a real demo. Press the car's RESET button to escape.

**10 · Challenge.** A small version of the real Science Olympiad Electric Vehicle event. Teams tape a course, guess distances, run, measure with a ruler, and adjust. Score them on how close they finish to the target and to a target time.

### Going further

- Ask students to sketch a drawing as a list of commands before coding it ("unplugged programming").
- Have pairs swap programs and predict what the other team's robot will do.
- Hold a mini competition: closest to a target point, fastest accurate square, best maze run.

## 5. Common mistakes

| Symptom | Likely cause | Fix |
|---|---|---|
| `expected ';' before …` when uploading | missing semicolon on the line **above** the error | add `;` |
| `'Robot' was not declared` / `'turnleft' …` | capital letters wrong | C++ is case-sensitive: `robot.turnLeft` |
| `expected '}' at end of input` | a `{` without a matching `}` | count braces; use **Tools → Auto Format** |
| Nothing happens after upload | waiting for START, or `begin()` missing | press START; check `robot.begin();` is first |
| The car repeats its moves forever | moves are in `loop()` | move them into `setup()` |
| `if` always runs | `=` instead of `==` | use `==` to compare |
| Shape comes out wrong with decimal angles | integer division `360 / 7` | write `360.0 / 7` |
| Car moves before the student lets go | no `waitForButton()` | add it after `begin()` |

## 6. "Oops!" messages

When a student makes a mistake the car can catch, it prints a message starting with `Oops!` in the Serial Monitor and turns the light red. The program keeps going (the bad command is skipped).

| Message | Meaning |
|---|---|
| `forward(-10): distance must be bigger than 0` | negative or zero distance; use `backward()` instead |
| `forward(5000): that's more than 10 meters!` | probably meant cm, not mm |
| `turnLeft(-90): use a positive angle…` | use the other turn command |
| `setSpeed(15): speed must be from 1 to 10` | speed was clamped to the nearest allowed value |
| `wait(-1): time can't be less than 0` | negative wait |
| `put robot.begin(); at the start of setup() first` | `begin()` missing or after other commands |
| `forward(30) stopped early … are the wheels slipping?` | wheel slip, a stalled motor, or a blocked car |
| `the robot has no gyro, so it can't measure turns` | gyro not detected at startup; check its wiring |

## 7. How the car works (for curious students)

- **Stepper motors** move in tiny fixed steps (1,600 per wheel turn), so the car knows how far it *told* the wheels to go.
- **Magnetic encoders** on each wheel measure how far the wheels *actually* turned. The car stops when the encoders say the distance is done.
- A **gyro** measures turning. It keeps the car pointing straight while driving, and turns are measured with it, not guessed.
- The car **ramps** its speed up and down so the wheels don't slip.
- **Mistakes don't add up.** Each turn is measured from where the car *should* be pointing, not where it ended up, so a square closes even after many turns.
- **Moving the car by hand.** While parked, the car ignores slow gyro drift, so a car picked up and put down drives the way it now points.

## 8. Teacher tools and settings

- **Teacher examples:** `Teacher_CarSetup` (Bluetooth name and PIN), `EngineTest` (straight driving), `SquareTest` (turns) and `ShiverTest` (motor smoothness). The test examples print a detailed report for every move.
- **Debug reports in any sketch:**
  ```cpp
  #include <internal/Engine.h>
  // in setup(), after robot.begin():
  evbot::tuning.debugPrint = true;
  ```
- **Car settings** (pins, wheel size, speed limits) are in `src/evbot_config.h`. Students never need to open it.
- **Calibration:** if the car was calibrated with the competition firmware (`PushReverseStepper`), EVBot reuses that distance calibration automatically.

## 9. Troubleshooting

| Problem | What to check |
|---|---|
| Serial Monitor shows nothing | baud rate 115200; correct port; press the car's RESET button |
| `Core dump data check failed` at startup | harmless; ignore it |
| Board restarts (`rst:0x1 (POWERON)`) during a run | power was lost: check the power switch and battery connector |
| Car drives backward on `forward()` or spins in circles | motor direction setting in `evbot_config.h`; ask the car's builder |
| Turns are consistently a bit short or long | gyro scale; note how far off and tell the car's builder |
| Distances are consistently off by a few % | needs distance calibration (competition firmware `CALIBRATE`) |
| Status LED flashing | battery below 11 V; recharge |
| Light stays amber | startup didn't finish; press RESET and keep the car still |
| Uploader page can't find the car | wireless switch on? wait ~7 s after power-on; the car must be running a program that calls `robot.begin()` |
| Uploader: "Can't start the update" | Arduino IDE **Tools → Partition Scheme → Default**, then upload once over USB |
| Uploader: "not a working program" | wrong file; choose the `.ino.bin`, not `.merged.bin` / `.bootloader.bin` |
| Robot runs the old program after upload | the student forgot **Export Compiled Binary** after changing the code |
