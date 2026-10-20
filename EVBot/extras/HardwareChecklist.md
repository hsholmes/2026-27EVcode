# EVBot Hardware Checklist

Use this before the first class with a car, after any repair, and whenever a car behaves oddly. The **Teacher_CarCheck** example (File → Examples → EVBot) runs most checks automatically and prints `OK` or `CHECK` for each.

## 1. Once per car (setup)

- [ ] Arduino IDE: **Tools → Board → ESP32S3 Dev Module**, **Tools → Partition Scheme → Default 4MB with spiffs**.
- [ ] Upload **Teacher_CarSetup** over USB with this car's name and PIN; put a sticker with both on the car.
- [ ] Upload **Teacher_CarCheck** over USB and go through sections 2–4 below.

## 2. Before each class (2 minutes per car)

| Check | How | Good | If not |
|---|---|---|---|
| Battery | CarCheck "Battery" line, or `robot.battery()` | 11 V or more; status LED not flashing | Charge or swap the NiMH pack |
| Wheels | Spin each wheel by hand with the car switched off | Turns freely, no wobble, set screw tight | Tighten the hub set screw |
| Tires | Look | Clean, not cracked | Wipe with a damp cloth; dusty tires slip |
| Wireless switch | WL_SW on; wireless LED lights ~1 s after power-on | LED on | Check the switch; without it there is no Bluetooth upload |
| Weight balance | Battery pack in the middle, between the wheels | Car drives straight with `forward()` | Move the pack to the centre - an unloaded wheel slips |

## 3. Teacher_CarCheck part 1 (automatic, no driving)

| Line | Means | If CHECK |
|---|---|---|
| Battery | Rail voltage ≥ 11 V | Charge the pack |
| Gyro (MPU-6050) found | The turning sensor answers on I2C | Check the gyro board's wires (SDA on GPIO 1, SCL on GPIO 2, power, GND) |
| Left / right wheel encoder magnet | Each AS5600 sees its magnet | Magnet gap should be ~1–2 mm and centred over the chip; check the encoder wires (left on SDA 41 / SCL 42, right shares the gyro bus) |
| Bluetooth on | Wireless switch on at power-up | Flip WL_SW on (Bluetooth starts within half a second) |
| Gyro reads still while parked | Less than 0.5 °/s when nobody touches the car | Keep the car still during power-up (it calibrates for 7 s); if it persists, the gyro board may be loose |
| Distance calibration | Shows the encoder calibration in use | If distances are off (section 4), calibrate with the competition firmware's `CALIBRATE` command |

## 4. Teacher_CarCheck part 2 (driving)

Mark the car's start spot with tape, give it ~1 m of space, press START, and follow the prompts.

| Test | Good | If not |
|---|---|---|
| forward(50) "wheels turned" | Both wheels 90–110 % | One wheel low = slipping tire, loose hub, or a motor skipping steps (check wiring, battery) |
| MEASURE 50 cm | 49–51 cm | Consistently long/short → distance calibration; varies → tire slip |
| Back on the start mark after backward(50) | Within ~1 cm | As above |
| Turns: 360 left + 360 right | Under 1° off | Gyro problem - repeat with the car untouched during start-up |
| LOOK: same direction after the turns | Same direction (by eye, against a floor tile line) | Turns consistently over/under-shooting means the gyro scale is off - note how many degrees and tell the car's builder |
| 40 cm square closes | Within ~2 cm, same direction | Combination of the above |
| LISTEN: beep | Audible beep | Not fatal - beeps use the stepper motors and are quiet on some drivers |

## 5. Quick fixes during class

| Problem | Fix |
|---|---|
| Car won't start, light stays amber | Keep it still; press RESET |
| Uploader can't connect | Wireless switch on? Close other uploader tabs; disconnect and connect again |
| Upload takes minutes | Open the uploader with `#debug`; tiny packets → disconnect and reconnect |
| Car drives backward or spins on `forward()` | Motor wiring swapped - car needs repair (see `LEFT/RIGHT_DIR_FORWARD` in `src/evbot_config.h`) |
| Board restarts by itself (`rst:0x1 (POWERON)`) | Power was lost: battery connector or power switch |
