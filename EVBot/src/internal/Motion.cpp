#include "Motion.h"

#include "../evbot_config.h"
#include "Encoders.h"
#include "Engine.h"
#include "Gyro.h"
#include "Steppers.h"

namespace evbot {
namespace motion {

using namespace config;

namespace {

// Step cap = this x the nominal steps for the distance. The encoders decide
// when a move is finished; the cap only stops the motors if they never do.
const float STEP_SAFETY_MARGIN = 1.3;

// The ramp is computed here in the control loop; the slew timer's own
// per-wheel slew runs this many times faster, so it only smooths between
// passes. If the slew timer ramped at the move accel itself, both wheels
// would climb in lockstep and steering would do nothing until cruise.
const float WHEEL_SLEW_FACTOR = 4.0;

// Stall recovery. A stepper pushed past its torque stalls and buzzes in
// place, and won't pick back up until the commanded speed drops far below
// where it stalled. Every STALL_WINDOW_MS each wheel's encoder travel is
// compared with the steps it was sent; if either turned less than
// STALL_RATIO of that, the ramp restarts from STALL_RESTART_FRACTION of the
// stall speed and the rest of the move is capped at STALL_CAP_FRACTION.
const unsigned long STALL_WINDOW_MS = 100;
const unsigned long STALL_HOLDOFF_MS = 400;  // let the motor re-sync before judging again
const long STALL_MIN_STEPS = 80;             // fewer steps (below ~800 st/s) are too few to judge
const float STALL_RATIO = 0.5;
const float STALL_RESTART_FRACTION = 0.3;
const float STALL_CAP_FRACTION = 0.85;

// Encoder trim that opposes the gyro correction is scaled from 100% at
// TRIM_FADE_START_DEG of heading error down to TRIM_OPPOSING_MIN at
// TRIM_FADE_END_DEG.
const float TRIM_FADE_START_DEG = 0.1;
const float TRIM_FADE_END_DEG = 0.4;
const float TRIM_OPPOSING_MIN = 0.1;

// The trim is low-pass filtered with this time constant before it reaches
// the wheels. Wheel tick difference builds up slowly, so the trim only needs
// to be slow - and any trim that tracks the heading error (the fade above)
// otherwise adds to the heading gain: near the fade the car steered ~7x
// harder than kpHeading and rocked at ~12 Hz when crawling.
const float TRIM_FILTER_S = 0.3;

// Turns: done when the gyro is within TURN_DONE_DEG of the target. The step
// cap assumes a track width of up to MAX_TRACK_WIDTH_M, wider than the car.
const float TURN_DONE_DEG = 0.05;
const float MAX_TRACK_WIDTH_M = 0.40;
const unsigned long TURN_SETTLE_MS = 100;

// Timeout = this x the time the move should take, plus TIMEOUT_EXTRA_MS.
const float TIMEOUT_FACTOR = 3.0;
const unsigned long TIMEOUT_EXTRA_MS = 5000;

struct StallWindow {
  unsigned long startMs;
  long stepsL, stepsR;
  long ticksL, ticksR;

  void reset(unsigned long atMs) {
    startMs = atMs;
    stepsL = steppers::leftSteps();
    stepsR = steppers::rightSteps();
    ticksL = encoders::leftTicks();
    ticksR = encoders::rightTicks();
  }
};

}  // namespace

MoveReport driveStraight(float distanceM, float cruiseMps, bool forward, float holdHeadingDeg) {
  MoveReport report = { MoveResult::Done, 0, 0, 0, 0, 0 };
  if (distanceM <= 0) return report;
  if (steppers::lockedOut()) {
    report.result = MoveResult::Halted;
    return report;
  }

  if (tuning.rezeroBeforeMove) gyro::rezero();

  const bool hasImu = gyro::available();
  const float dirSign = forward ? 1.0 : -1.0;
  const float targetM = distanceM * tuning.distFactor;  // in encoder distance
  const float cruise = constrain(cruiseMps * STEPS_PER_METER, MIN_SAFE_SPEED, MAX_SPEED);
  const float accelRate = max(tuning.accelMps2, 0.05f) * STEPS_PER_METER;  // st/s^2
  const long stepCap = (long)(targetM * STEPS_PER_METER * STEP_SAFETY_MARGIN) + 1;

  // The line to drive along: holdHeadingDeg, not wherever the car happens to
  // point now, so small turn errors don't add up from move to move.
  const float holdHeading = holdHeadingDeg;
  const long startTicksL = encoders::leftTicks();
  const long startTicksR = encoders::rightTicks();
  float crossTrackM = 0.0;  // sideways offset from the start line, + = left
  float lastLegM = 0.0;

  // How long the move should take (ramp up + cruise + ramp down), for the timeout
  float cruiseS = cruise / accelRate;
  float expectedS = targetM / (cruise / STEPS_PER_METER) + cruiseS;
  unsigned long timeoutMs = (unsigned long)(expectedS * 1000.0 * TIMEOUT_FACTOR) + TIMEOUT_EXTRA_MS;

  steppers::setDirections(forward, forward);
  steppers::enable(true);
  steppers::start(stepCap, stepCap, accelRate * WHEEL_SLEW_FACTOR);

  const unsigned long startMs = millis();
  unsigned long rampStartMs = startMs;
  float rampFloorSpeed = MIN_SAFE_SPEED;
  float stallSpeedCap = cruise;
  StallWindow win;
  win.reset(startMs);
  unsigned long lastPrintMs = 0;
  float encCorrFiltered = 0.0;
  unsigned long lastPassUs = micros();

  for (;;) {
    // Encoder distance - the stop signal, and what the ramp-down uses.
    long dLeftTicks = labs(encoders::leftTicks() - startTicksL);
    long dRightTicks = labs(encoders::rightTicks() - startTicksR);
    float avgLegM = (dLeftTicks + dRightTicks) / 2.0 / tuning.ticksPerMeter;
    float headingRel = gyro::headingDeg() - holdHeading;

    if (avgLegM >= targetM) {
      report.result = MoveResult::Done;
      break;
    }
    if (steppers::lockedOut()) {
      report.result = MoveResult::Halted;
      break;
    }
    if (!steppers::running()) {
      report.result = MoveResult::StepCap;
      break;
    }
    if (millis() - startMs > timeoutMs) {
      report.result = MoveResult::Timeout;
      break;
    }

    // Stall check - see STALL_WINDOW_MS.
    bool snapL = false, snapR = false;
    long sinceWinMs = (long)(millis() - win.startMs);
    if (sinceWinMs < 0) {
      // Holdoff after a stall: keep re-basing so the next window starts
      // when the holdoff ends.
      win.reset(win.startMs);
    } else if (sinceWinMs >= (long)STALL_WINDOW_MS) {
      long winStepsL = steppers::leftSteps() - win.stepsL;
      long winStepsR = steppers::rightSteps() - win.stepsR;
      long winTicksL = labs(encoders::leftTicks() - win.ticksL);
      long winTicksR = labs(encoders::rightTicks() - win.ticksR);
      bool stalledL = winStepsL >= STALL_MIN_STEPS && winTicksL < STALL_RATIO * winStepsL * TICKS_PER_STEP;
      bool stalledR = winStepsR >= STALL_MIN_STEPS && winTicksR < STALL_RATIO * winStepsR * TICKS_PER_STEP;
      if (stalledL || stalledR) {
        float stallSpeed = max(steppers::leftSpeed(), steppers::rightSpeed());
        stallSpeedCap = max(min(stallSpeedCap, stallSpeed * STALL_CAP_FRACTION), MIN_SAFE_SPEED);
        rampFloorSpeed = max(stallSpeed * STALL_RESTART_FRACTION, MIN_SAFE_SPEED);
        rampStartMs = millis();
        report.stalls++;
        win.reset(millis() + STALL_HOLDOFF_MS);
        Serial.print("[EVBot] Wheel stalled (");
        Serial.print(stalledL ? "L" : "");
        Serial.print(stalledR ? "R" : "");
        Serial.print(") at ");
        Serial.print(stallSpeed, 0);
        Serial.println(" st/s - slowing down");
        snapL = stalledL;
        snapR = stalledR;
      } else {
        win.reset(millis());
      }
    }

    // Ramp up by time since the start (or the last stall), ramp down over
    // the last stretch of distance at the same rate (v^2 = 2*a*d). min():
    // on a short move the ramp-down can begin before the ramp-up has
    // finished - never jump up to meet the ramp-down curve.
    float topSpeed = min(cruise, stallSpeedCap);
    float rampElapsedS = (millis() - rampStartMs) / 1000.0;
    float rampedSpeed = constrain(rampFloorSpeed + accelRate * rampElapsedS, MIN_SAFE_SPEED, topSpeed);
    float remainingM = targetM - avgLegM;
    float accelMps2 = accelRate / STEPS_PER_METER;
    float downSpeed = sqrt(2.0 * accelMps2 * remainingM) * STEPS_PER_METER;
    rampedSpeed = min(rampedSpeed, max(downSpeed, MIN_SAFE_SPEED));

    // Cross-track: integrate sideways drift from encoder travel and gyro
    // heading. Holding the heading alone leaves the car parallel to the
    // line but offset from it after any early swerve; aiming slightly back
    // toward the line (target heading ~ -offset) actually returns it.
    float ds = avgLegM - lastLegM;
    lastLegM = avgLegM;
    if (hasImu) crossTrackM += dirSign * ds * sin(headingRel * DEG_TO_RAD);
    float targetHeading = constrain(-dirSign * tuning.kx * crossTrackM,
                                    -tuning.maxXtHeadingDeg, tuning.maxXtHeadingDeg);

    // Reversing flips which wheel must speed up to correct the same
    // physical yaw, so the correction sign flips with it.
    float headingError = headingRel - targetHeading;
    float kpCorrection = hasImu ? dirSign * tuning.kpHeading * headingError : 0.0;

    // Encoder balance trim: diffTicks > 0 means the left wheel has turned
    // more, so slow it down and speed the right one up.
    long diffTicks = dLeftTicks - dRightTicks;
    float encCorr = constrain(-tuning.kpEnc * (float)diffTicks, -tuning.maxEncCorr, tuning.maxEncCorr);
    // While the gyro is actively steering, don't let the trim fight the turn.
    // Faded in smoothly over TRIM_FADE_START..END degrees of error, then
    // filtered (TRIM_FILTER_S): the old on/off switch at 0.2 deg flipped the
    // trim between 100% and 10% whenever the error sat near 0.2, which at
    // low speed swapped which wheel was faster ~25 times a second - the car
    // shivered while slowing down.
    if (hasImu && ((kpCorrection > 0.0f && encCorr < 0.0f) || (kpCorrection < 0.0f && encCorr > 0.0f))) {
      float fade = constrain((fabs(headingError) - TRIM_FADE_START_DEG) / (TRIM_FADE_END_DEG - TRIM_FADE_START_DEG), 0.0f, 1.0f);
      encCorr *= 1.0f - (1.0f - TRIM_OPPOSING_MIN) * fade;
    }
    unsigned long nowUs = micros();
    float passS = (nowUs - lastPassUs) / 1000000.0;
    lastPassUs = nowUs;
    encCorrFiltered += (encCorr - encCorrFiltered) * passS / (TRIM_FILTER_S + passS);
    encCorr = encCorrFiltered;

    float leftSpd = rampedSpeed + kpCorrection + encCorr;
    float rightSpd = rampedSpeed - kpCorrection - encCorr;
    steppers::setTargets(constrain(leftSpd, rampedSpeed * 0.8f, rampedSpeed * 1.2f),
                         constrain(rightSpd, rampedSpeed * 0.8f, rampedSpeed * 1.2f));
    // A stalled wheel isn't turning anyway, so drop its command straight
    // to the restart speed. Only after the new targets above.
    steppers::snapDown(snapL, snapR);

    if (tuning.debugPrint && millis() - lastPrintMs >= tuning.debugIntervalMs) {
      lastPrintMs = millis();
      Serial.printf("[EVBot] %s t %lu dist %.3f/%.3f m  spd %.0f  L %.0f R %.0f  heading %+.2f  corr %+.0f  enc %+.0f  xt %+.0f mm\n",
                    forward ? "FWD" : "REV", millis() - startMs, avgLegM, targetM, rampedSpeed,
                    steppers::leftSpeed(), steppers::rightSpeed(), headingRel, kpCorrection, encCorr,
                    crossTrackM * 1000.0);
    }

    delay(1);
  }

  steppers::stop();
  report.timeMs = millis() - startMs;
  long dL = labs(encoders::leftTicks() - startTicksL);
  long dR = labs(encoders::rightTicks() - startTicksR);
  report.distanceM = (dL + dR) / 2.0 / tuning.ticksPerMeter / tuning.distFactor;
  report.headingErrDeg = gyro::headingDeg() - holdHeading;
  report.crossTrackMm = crossTrackM * 1000.0;
  return report;
}

MoveReport spin(float targetHeadingDeg, float maxWheelMps) {
  MoveReport report = { MoveResult::Done, 0, 0, 0, 0, 0 };
  if (!gyro::available()) {
    report.result = MoveResult::NoGyro;
    return report;
  }
  if (steppers::lockedOut()) {
    report.result = MoveResult::Halted;
    return report;
  }

  if (tuning.rezeroBeforeMove) gyro::rezero();

  const float startHeading = gyro::headingDeg();
  const float turnDeg = targetHeadingDeg - startHeading;
  if (fabs(turnDeg) <= TURN_DONE_DEG) {
    report.headingErrDeg = -turnDeg;
    return report;
  }
  const bool left = turnDeg > 0;  // + heading = nose turned left

  const float cruise = constrain(maxWheelMps * STEPS_PER_METER, MIN_SAFE_SPEED, MAX_SPEED);
  const float accelRate = max(tuning.accelMps2, 0.05f) * STEPS_PER_METER;
  // The gyro decides when the turn is done; the cap (sized for a car much
  // wider than this one) only stops the wheels if it never says so.
  const float capArcM = MAX_TRACK_WIDTH_M / 2.0 * fabs(turnDeg) * DEG_TO_RAD;
  const long stepCap = (long)(capArcM * STEPS_PER_METER * STEP_SAFETY_MARGIN) + 400;
  const unsigned long timeoutMs = (unsigned long)(capArcM / (MIN_SAFE_SPEED / STEPS_PER_METER) * 1000.0) + TIMEOUT_EXTRA_MS;

  // Spin in place: turning left = left wheel backward, right wheel forward.
  steppers::setDirections(!left, left);
  steppers::enable(true);
  steppers::start(stepCap, stepCap, accelRate * WHEEL_SLEW_FACTOR);

  const long startTicksL = encoders::leftTicks();
  const long startTicksR = encoders::rightTicks();
  const unsigned long startMs = millis();
  unsigned long lastPrintMs = 0;

  for (;;) {
    float heading = gyro::headingDeg();
    float remainingDeg = left ? targetHeadingDeg - heading : heading - targetHeadingDeg;

    if (remainingDeg <= TURN_DONE_DEG) {
      report.result = MoveResult::Done;
      break;
    }
    if (steppers::lockedOut()) {
      report.result = MoveResult::Halted;
      break;
    }
    if (!steppers::running()) {
      report.result = MoveResult::StepCap;
      break;
    }
    if (millis() - startMs > timeoutMs) {
      report.result = MoveResult::Timeout;
      break;
    }

    // Ramp up by time; slow down in proportion to the angle left, so the
    // car creeps onto the target instead of overshooting it. No track
    // width needed - the gyro measures the turn directly.
    float rampElapsedS = (millis() - startMs) / 1000.0;
    float speed = constrain(MIN_SAFE_SPEED + accelRate * rampElapsedS, MIN_SAFE_SPEED, cruise);
    speed = min(speed, max(tuning.turnKp * remainingDeg, MIN_SAFE_SPEED));
    steppers::setTargets(speed, speed);

    if (tuning.debugPrint && millis() - lastPrintMs >= tuning.debugIntervalMs) {
      lastPrintMs = millis();
      Serial.printf("[EVBot] TURN %s t %lu heading %+.2f -> %+.2f  left %.2f deg  spd %.0f  L %.0f R %.0f\n",
                    left ? "L" : "R", millis() - startMs, heading, targetHeadingDeg, remainingDeg, speed,
                    steppers::leftSpeed(), steppers::rightSpeed());
    }

    delay(1);
  }

  steppers::stop();
  report.timeMs = millis() - startMs;
  long dL = labs(encoders::leftTicks() - startTicksL);
  long dR = labs(encoders::rightTicks() - startTicksR);
  report.distanceM = (dL + dR) / 2.0 / tuning.ticksPerMeter;  // wheel travel during the spin
  delay(TURN_SETTLE_MS);  // let the car stop rocking before reading where it ended up
  report.headingErrDeg = gyro::headingDeg() - targetHeadingDeg;
  return report;
}

}  // namespace motion
}  // namespace evbot
