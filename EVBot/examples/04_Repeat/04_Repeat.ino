// ============================================================
// Lesson 4: Repeat
//
// You will learn:
//   - a "for loop": a way to repeat code without writing it again
//
// Lesson 3 needed 8 lines for a square. With a loop it needs 2!
// ============================================================

#include <EVBot.h>

EVBot robot;

void setup() {
  robot.begin();
  robot.waitForButton();

  // A for loop has three parts, inside the ( ):
  //   int i = 0    start counting at 0
  //   i < 4        keep going while i is less than 4
  //   i++          add 1 to i after each time
  // So the code inside { } runs 4 times: i = 0, 1, 2, 3.
  for (int i = 0; i < 4; i++) {
    robot.forward(30);
    robot.turnLeft(90);
  }

  robot.light(GREEN);
}

void loop() {
}

// ============================================================
// Try this!
//   1. Change the 4 to 8. What happens? (Two laps!)
//   2. Make a triangle with a loop: 3 times, turn 120 degrees.
//   3. Make a hexagon: 6 sides, turn 60 degrees.
//   4. Add robot.say(i); inside the loop to see the count.
//   5. Make the light change color on every side. Hint: put
//      robot.light(...) inside the loop, before forward().
//   6. Challenge: put a loop INSIDE a loop to drive 3 squares.
// ============================================================
