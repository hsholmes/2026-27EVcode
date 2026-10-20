// Teacher_CarCheck - checks a car is ready for class. Teachers only.
//
// Open the Serial Monitor (115200) or the EVBot Uploader page to read the
// results. Part 1 runs by itself as soon as the car starts. Part 2 drives:
// put the car on the floor with ~1 m of space, mark its start spot with tape,
// and press START. Write down every CHECK line and fix it before class - see
// extras/HardwareChecklist.md for what each one means.

#include <EVBot.h>
#include <evbot_config.h>
#include <internal/Board.h>
#include <internal/Encoders.h>
#include <internal/Engine.h>
#include <internal/Gyro.h>
#include <internal/Steppers.h>
#include <internal/Wireless.h>

EVBot robot;
int problems = 0;

void result(bool ok, const String& what) {
  if (!ok) problems++;
  robot.say(String(ok ? "  OK     " : "  CHECK  ") + what);
}

// How far each wheel turned compared with how far it was told to (in %).
// Both near 100 = no slipping or skipped steps.
void wheelTracking(long ticksL0, long ticksR0, const char* move) {
  float ticksPerStep = evbot::config::TICKS_PER_STEP;
  float l = 100.0 * labs(evbot::encoders::leftTicks() - ticksL0) / (evbot::steppers::leftSteps() * ticksPerStep);
  float r = 100.0 * labs(evbot::encoders::rightTicks() - ticksR0) / (evbot::steppers::rightSteps() * ticksPerStep);
  result(l > 90 && l < 110 && r > 90 && r < 110,
         String(move) + ": wheels turned L " + String(l, 0) + "% / R " + String(r, 0) + "% of the steps sent");
}

void setup() {
  robot.begin();
  robot.say("===== EVBot car check =====");
  robot.say(String("Car name: ") + evbot::wireless::carName());

  // ---------- Part 1: parts (no driving) ----------
  robot.say("--- Part 1: parts ---");
  float volts = robot.battery();
  result(volts >= evbot::config::LOW_VOLTAGE_THRESHOLD, String("Battery ") + String(volts, 2) + " V (want 11 V or more)");
  result(evbot::gyro::available(), "Gyro (MPU-6050) found");
  result(evbot::encoders::leftMagnetOk(), "Left wheel encoder magnet detected");
  result(evbot::encoders::rightMagnetOk(), "Right wheel encoder magnet detected");
  result(evbot::wireless::running(), "Bluetooth on (wireless switch WL_SW on)");

  // A parked gyro should read (almost) zero turning.
  float worst = 0;
  for (int i = 0; i < 50; i++) {
    worst = max(worst, (float)fabs(evbot::gyro::rateDps()));
    delay(20);
  }
  result(worst < 0.5, String("Gyro reads still while parked (") + String(worst, 2) + " deg/s, want under 0.5)");
  robot.say(String("Distance calibration: ") + String(evbot::tuning.ticksPerMeter, 0) + " ticks/m (default " +
            String(evbot::config::DEFAULT_TICKS_PER_METER, 0) + ")");

  // ---------- Part 2: driving ----------
  robot.say("--- Part 2: driving - put the car on its start mark, press START ---");
  robot.waitForButton();
  robot.setSpeed(MEDIUM);

  long l0 = evbot::encoders::leftTicks(), r0 = evbot::encoders::rightTicks();
  robot.forward(50);
  wheelTracking(l0, r0, "forward(50)");
  robot.say("  MEASURE: the car should have moved 50 cm (+-1 cm). Then press START.");
  robot.waitForButton();

  l0 = evbot::encoders::leftTicks();
  r0 = evbot::encoders::rightTicks();
  robot.backward(50);
  wheelTracking(l0, r0, "backward(50)");
  robot.say("  LOOK: the car should be back on its start mark.");

  float h0 = robot.heading();
  robot.turnLeft(360);
  robot.turnRight(360);
  float drift = robot.heading() - h0;
  result(fabs(drift) < 1.0, String("Turns: 360 left + 360 right ended ") + String(drift, 2) + " deg off (want under 1)");
  robot.say("  LOOK: the car should point exactly the way it did before the turns.");

  for (int i = 0; i < 4; i++) {
    robot.forward(40);
    robot.turnLeft(90);
  }
  robot.say("  LOOK: after the 40 cm square the car should be on its start mark, pointing the same way.");

  robot.beep(660, 0.3);
  robot.say("  LISTEN: you should have heard a beep.");

  robot.say("===== Done =====");
  if (problems == 0) {
    robot.say("All automatic checks OK. Also check the MEASURE / LOOK / LISTEN lines above.");
    robot.light(GREEN);
  } else {
    robot.say(String(problems) + " CHECK line(s) - see extras/HardwareChecklist.md");
    robot.light(RED);
  }
}

void loop() {
}
