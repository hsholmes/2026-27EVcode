# EVBot Cheat Sheet

## Every program starts like this

```cpp
#include <EVBot.h>
EVBot robot;

void setup() {
  robot.begin();          // ALWAYS first - keep the robot still ~7 seconds
  robot.waitForButton();  // wait for START before moving
  // your commands go here - they run once, top to bottom
}

void loop() {
  // commands here repeat forever
}
```

Each command ends with a **semicolon** `;`. Capital letters matter: `turnLeft`, not `turnleft`.

## Moving

| Command | What it does |
|---|---|
| `robot.forward(30);` | drive forward 30 **cm** |
| `robot.backward(30);` | drive backward 30 cm |
| `robot.turnLeft(90);` | turn left 90 **degrees** (a quarter turn) |
| `robot.turnRight(90);` | turn right 90 degrees |
| `robot.setSpeed(5);` | speed from 1 (slow) to 10 (fast), or `SLOW` `MEDIUM` `FAST` |
| `robot.wait(1.5);` | do nothing for 1.5 **seconds** |
| `robot.stop();` | stop the wheels |

The robot finishes each move before the next line runs.

## Lights, button and messages

| Command | What it does |
|---|---|
| `robot.light(GREEN);` | `RED` `ORANGE` `YELLOW` `GREEN` `CYAN` `BLUE` `PURPLE` `PINK` `WHITE` `OFF` |
| `robot.light(255, 100, 0);` | mix a color: red, green, blue (0–255 each) |
| `robot.waitForButton();` | wait until START is pressed |
| `robot.buttonPressed()` | `true` while START is held down |
| `robot.say("Hi!");` | show a message in the Serial Monitor (115200 baud) |
| `robot.say("Distance", robot.distance());` | show a name and a number |

## Senses (they give you a number)

| Command | Gives you |
|---|---|
| `robot.distance()` | cm driven since the start (forward adds, backward subtracts) |
| `robot.heading()` | degrees turned since the start (right = +, left = −) |
| `robot.battery()` | battery volts (about 12 when full) |

## Level 3: Explorer

| Command | What it does |
|---|---|
| `robot.drive(5, 5);` | set each wheel's speed (-10 to 10) and **keep going** - the next line runs right away |
| `robot.spinLeft();` `robot.spinRight();` | spin on the spot and keep spinning |
| `robot.stop();` | stop after `drive` or a spin |
| `robot.arcLeft(30, 90);` | curve left along a circle of radius 30 cm, turning 90 degrees |
| `robot.arcRight(30, 90);` | the same, to the right |
| `robot.resetDistance();` `robot.resetHeading();` | start counting from 0 again |
| `robot.beep();` `robot.beep(440, 0.5);` | make a sound (frequency in Hz, seconds) |

```cpp
robot.drive(5, 5);                 // start driving...
while (robot.distance() < 80) {     // ...and keep checking
  robot.wait(0.05);
}
robot.stop();
```

## C++ building blocks

```cpp
int sides = 4;            // whole-number variable
float angle = 360.0 / 4;  // decimal variable

for (int i = 0; i < 4; i++) {   // repeat 4 times
  robot.forward(30);
}

while (robot.distance() < 100) {  // repeat while true
  robot.forward(10);
}

if (robot.buttonPressed()) {      // decide
  robot.light(GREEN);
} else {
  robot.light(RED);
}

void square(int size) {           // make your own command
  for (int i = 0; i < 4; i++) {
    robot.forward(size);
    robot.turnLeft(90);
  }
}
```

| Compare | Means | Combine | Means |
|---|---|---|---|
| `==` | is equal to | `&&` | and |
| `!=` | is not equal to | `\|\|` | or |
| `<` `>` | less / greater than | `!` | not |
| `<=` `>=` | less / greater or equal | | |

## Shapes

| Shape | Sides | Turn at each corner |
|---|---|---|
| Triangle | 3 | 120 |
| Square | 4 | 90 |
| Pentagon | 5 | 72 |
| Hexagon | 6 | 60 |
| Any shape | n | 360 / n |

## Upload without a cable

1. Wireless switch **on**.
2. Arduino IDE: **Sketch → Export Compiled Binary**.
3. EVBot Uploader page: **Connect** → PIN → **Choose file** (`.ino.bin`) → **Upload**.

Changed your code? **Export again**, then Upload.

Opened a lesson from **File → Examples**? Do **File → Save As** first and work in your own copy, otherwise you change the lesson for everyone and the uploader may still be pointing at your old copy.

## Help!

- **Red light + "Oops!" in the Serial Monitor**: read the message. It tells you what went wrong.
- **Nothing happens**: did you press START? Is `robot.begin();` the first line in `setup()`?
- **Robot drives the wrong way or spins**: tell your teacher.
- **Error when you click Upload**: look for a missing `;`, a missing `}`, or a capital letter in the wrong place.
