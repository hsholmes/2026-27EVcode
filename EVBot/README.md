# EVBot

Learn C++ by driving a real robot. EVBot is an Arduino library for grade 4–7 students, built for the Science Olympiad Electric Vehicle car (ESP32-S3).

```cpp
#include <EVBot.h>
EVBot robot;

void setup() {
  robot.begin();
  robot.waitForButton();
  for (int i = 0; i < 4; i++) {
    robot.forward(30);    // centimeters
    robot.turnLeft(90);   // degrees
  }
  robot.light(GREEN);
}

void loop() {
}
```

The car drives straight and turns accurately by itself, using its gyro and wheel encoders. Students only write the steps.

## What's inside

| | |
|---|---|
| `examples/01_HelloLight` … `10_Challenge_PushReverse` | ten student lessons, one idea each, with "Try this!" challenges |
| `extras/CheatSheet.md` | one-page command reference for students |
| `extras/TeacherGuide.md` | setup, lesson plans, common mistakes, troubleshooting |
| `extras/EVBotUploader.html` | upload programs and see `robot.say()` messages over Bluetooth (Chrome / Edge) |
| `examples/Teacher_CarSetup` | give each car its Bluetooth name and PIN |
| `examples/EngineTest`, `SquareTest`, `ShiverTest` | teacher / developer test sketches |
| `src/evbot_config.h` | pins and car constants |

## Commands

**Moving:** `forward(cm)`, `backward(cm)`, `turnLeft(deg)`, `turnRight(deg)`, `setSpeed(1–10)`, `wait(sec)`, `stop()`

**Senses:** `light(color)`, `buttonPressed()`, `waitForButton()`, `heading()`, `distance()`, `battery()`, `say(...)`

See the [cheat sheet](extras/CheatSheet.md) for details.

## Requirements

- Arduino IDE 2.x with **esp32 by Espressif Systems** 3.x (tested 3.3.12), board **ESP32S3 Dev Module**
- No other libraries

Setup steps are in the [teacher guide](extras/TeacherGuide.md#1-setting-up-once-per-computer).
