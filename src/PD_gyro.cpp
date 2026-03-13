#pragma region VEXcode Generated Robot Configuration
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "vex.h"

using namespace vex;

// Brain should be defined by default
brain Brain;


// Robot configuration code.
gyro        Gyro2       = gyro(PORT2);
motor       right_motor = motor(PORT8, false);
motor       left_motor  = motor(PORT7, true);   // inverted — left side faces opposite direction
touchled    TouchLED3   = touchled(PORT3);
motor       ArmMotorA   = motor(PORT9, false);
motor       ArmMotorB   = motor(PORT10, false);
motor_group four_bar    = motor_group(ArmMotorA, ArmMotorB);
motor       M_180       = motor(PORT1, false);
motor       Kamasha     = motor(PORT4, false);

// Seeds the RNG using brain uptime for entropy
// NOTE: Brain.Timer.system() unit is unconfirmed — verify before removing * 1000
void initializeRandomSeed() {
  wait(100, msec);
  srand(static_cast<unsigned int>(vex::timer::system() * 1000));
}

void vexcodeInit() {
  initializeRandomSeed();
}

#pragma endregion VEXcode Generated Robot Configuration

using namespace vex;

// ── Adaptive Config ───────────────────────────────────────────────────────────
// Holds all tuning constants as one swappable block.
// Call selectConfig() at the start of configureRobot() to pick a preset.
typedef struct {
  float Kp_straight, Kd_straight;
  float Kp_dist, Ki_dist, Kd_dist;
  float Kp_PIVOT, Kd_PIVOT;
  float Kp_SPIN,  Kd_SPIN;
} RobotConfig;

// TODO: tune the weighted presets on the actual robot before competition
constexpr RobotConfig CONFIG_NO_WEIGHT = { 1.5f, 4.0f,  0.5f, 0.0f, 0.2f,  0.6f, 4.0f,  1.35f, 8.0f  };
constexpr RobotConfig CONFIG_2_WEIGHTS = { 1.3f, 4.5f,  0.4f, 0.0f, 0.3f,  0.5f, 4.5f,  1.20f, 9.0f  };
constexpr RobotConfig CONFIG_4_WEIGHTS = { 1.1f, 5.0f,  0.3f, 0.0f, 0.4f,  0.4f, 5.0f,  1.00f, 10.0f };

RobotConfig cfg; // active config — all functions read gains from here

void selectConfig(const RobotConfig* preset) {
  cfg = *preset;
}

// ── Global Variables ──────────────────────────────────────────────────────────
float kAngleToleranceDeg = 1.0; // acceptable error band  // <- Modifiable

// ── Function Prototypes ───────────────────────────────────────────────────────
void pivotRightToAngle(float targetAngle, int timeoutMs = 3000);
void pivotLeftToAngle(float targetAngle, int timeoutMs = 3000);
void spinToAngle(float targetAngle, int timeoutMs = 3000);
void driveStraight(float velocity, float targetDistanceDegrees, float targetAngle, int timeoutMs = 3000);

// ── Helpers ───────────────────────────────────────────────────────────────────

// Wraps any angle into [-180, +180] so PD controllers always take the shortest path
float normalizeAngle(float angle) {
  if (angle >  180.0f) angle -= 360.0f;
  if (angle < -180.0f) angle += 360.0f;
  return angle;
}

// Symmetric clamp: keeps v inside [-limit, +limit]
static  float clamp(float v, float limit) {
  while (v >  limit) return  limit;
  while (v < -limit) return -limit;
  return v;
}

// ── Pivot (one wheel only) ────────────────────────────────────────────────────
// Rotates around the stopped wheel.
//   pivotMotor : the motor that actually spins (pass left_motor or right_motor)
//   spinDir    : +1 = spin forward when error > 0
//                -1 = spin reverse when error > 0
void pivotToAngle(const float targetAngle, motor &pivotMotor, const int spinDir, const int timeoutMs = 3000) {
  right_motor.setStopping(hold);
  left_motor.setStopping(hold);
  right_motor.stop();
  left_motor.stop();

  float headingError         = 0.0;
  float previousHeadingError = normalizeAngle(targetAngle - static_cast<float>(Gyro2.rotation()));
  int   settledCount         = 0;
  uint32_t startTime         = vex::timer::system();  // ms, confirmed from SDK

  while (true) {
    headingError = normalizeAngle(targetAngle - static_cast<float>(Gyro2.rotation()));

    // PD output — derivative term dampens overshoot
    float velocity = clamp(
      headingError * cfg.Kp_PIVOT + cfg.Kd_PIVOT * (headingError - previousHeadingError),
      100.0f
    );

    pivotMotor.setVelocity(fabs(velocity), percent);
    pivotMotor.spin((velocity * static_cast<float>(spinDir) > 0) ? forward : reverse);

    // Must stay within tolerance for 5 consecutive loops (~100 ms) before stopping
    if (fabsf(headingError) <= kAngleToleranceDeg) {
      if (++settledCount > 5) break;
    } else {
      settledCount = 0;
    }

    // Timeout check using Brain timer — no manual counter needed
    if (static_cast<int32_t>(vex::timer::system() - startTime) >= timeoutMs) break;

    previousHeadingError = headingError;
    wait(20, msec);
  }
  left_motor.stop();
  right_motor.stop();
}

// Pivot using the left wheel only (robot rotates rightward)
void pivotRightToAngle(const float targetAngle, const int timeoutMs) { pivotToAngle(targetAngle, left_motor,  -1, timeoutMs); }
// Pivot using the right wheel only (robot rotates leftward)
void pivotLeftToAngle (const float targetAngle, const int timeoutMs) { pivotToAngle(targetAngle, right_motor, +1, timeoutMs); }

// ── Spin (both wheels, opposite directions) ───────────────────────────────────
// Turns in place around the robot's center. Faster than pivoting but uses both motors.
void spinToAngle(const float targetAngle, const int timeoutMs) {
  right_motor.setStopping(hold);
  left_motor.setStopping(hold);
  right_motor.stop();
  left_motor.stop();

  float headingError         = 0.0;
  float previousHeadingError = normalizeAngle(targetAngle - static_cast<float>(Gyro2.rotation()));
  int   settledCount         = 0;
  int   elapsedTimer         = 0;

  while (true) {
    headingError = normalizeAngle(targetAngle - static_cast<float>(Gyro2.rotation()));

    // PD output
    float velocity = clamp(
      headingError * cfg.Kp_SPIN + cfg.Kd_SPIN * (headingError - previousHeadingError),
      100.0f
    );

    left_motor.setVelocity(fabsf(velocity),  percent);
    right_motor.setVelocity(fabsf(velocity), percent);

    // Positive velocity → turn one way; negative → turn the other
    if (velocity > 0) { right_motor.spin(forward);  left_motor.spin(reverse); }
    else              { right_motor.spin(reverse);  left_motor.spin(forward); }

    if (fabsf(headingError) <= kAngleToleranceDeg) {
      settledCount++;
      if (settledCount > 5) break;
    } else {
      settledCount = 0;
    }

    // Timeout check — gives up if target is unreachable
    elapsedTimer += 20;
    if (elapsedTimer >= timeoutMs) break;

    previousHeadingError = headingError;
    wait(20, msec);
  }
  left_motor.stop(hold);
  right_motor.stop(hold);
}

// ── Drive straight ────────────────────────────────────────────────────────────
// Drives to targetDistanceDegrees (motor shaft degrees, not wheel travel distance).
// velocity sign sets direction; targetAngle keeps the heading locked via PD.
// timeoutMs : hard cutoff so the robot never gets stuck forever
void driveStraight(const float velocity, const float targetDistanceDegrees, const float targetAngle,
                   const int timeoutMs) {
  constexpr int settleTimeMs = 150;  // ms to stay within threshold before declaring done
  right_motor.setStopping(hold);
  left_motor.setStopping(hold);
  right_motor.resetPosition();
  left_motor.resetPosition();

  // Flip target sign to match velocity direction
  float directionSign = (velocity >= 0) ? 1.0 : -1.0;
  float actualTarget  = directionSign * fabsf(targetDistanceDegrees);
  float MAX_DRIVE     = fabsf(velocity);

  float distError     = 0.0;
  float prevDistError = 0.0;
  float distIntegral  = 0.0;

  float headingError     = 0.0;
  float prevHeadingError = normalizeAngle(targetAngle - static_cast<float>(Gyro2.rotation()));

  constexpr float SETTLE_THRESHOLD = 2.0;

  int settleTimer  = 0;
  int elapsedTimer = 0;

  while (true) {
    constexpr float INTEGRAL_LIMIT   = 25.0;
    // Average both motors so one slipping doesn't throw off distance tracking
    float averageDistance = (static_cast<float>(right_motor.position(degrees)) + static_cast<float>(left_motor.position(degrees))) / 2.0f;
    // ── Distance PID ──
    distError     = actualTarget - averageDistance;
    distIntegral += distError;
    if (distIntegral >  INTEGRAL_LIMIT) distIntegral =  INTEGRAL_LIMIT;  // clamp to prevent windup
    if (distIntegral < -INTEGRAL_LIMIT) distIntegral = -INTEGRAL_LIMIT;

    float distOutput = clamp(
      cfg.Kp_dist * distError + cfg.Ki_dist * distIntegral + cfg.Kd_dist * (distError - prevDistError),
      MAX_DRIVE
    );
    prevDistError = distError;

    // ── Heading PD (keeps robot going straight) ──
    headingError = normalizeAngle(targetAngle - static_cast<float>(Gyro2.rotation()));

    float headingCorrection = cfg.Kp_straight * headingError
                            + cfg.Kd_straight * (headingError - prevHeadingError);
    prevHeadingError = headingError;

    // Flip correction sign when reversing so heading correction stays correct
    float correctionSign = (velocity >= 0) ? 1.0f : -1.0f;

    float rightVel = clamp(distOutput + headingCorrection * correctionSign, 100.0f);
    float leftVel  = clamp(distOutput - headingCorrection * correctionSign, 100.0f);

    right_motor.setVelocity(fabsf(rightVel), percent);
    left_motor.setVelocity (fabsf(leftVel),  percent);
    right_motor.spin(rightVel > 0 ? forward : reverse);
    left_motor.spin (leftVel  > 0 ? forward : reverse);

    // ── Settle check ──
    if (fabsf(distError) < SETTLE_THRESHOLD) {
      settleTimer += 20;
      if (settleTimer >= settleTimeMs) break;
    } else {
      settleTimer = 0;
    }

    // ── Timeout check ──
    elapsedTimer += 20;
    if (elapsedTimer >= timeoutMs) break;

    wait(20, msec);
  }
  right_motor.stop(hold);
  left_motor.stop(hold);
}

// ── Setup ─────────────────────────────────────────────────────────────────────
int configureRobot() {

  TouchLED3.setColor(red);  // red = not ready yet

  // Pick weight configuration here before anything runs
  selectConfig(&CONFIG_NO_WEIGHT);

  // All motors default to hold so they don't backdrive when stopped
  right_motor.setStopping(hold);
  left_motor.setStopping(hold);
  four_bar.setStopping(hold);
  Kamasha.setStopping(hold);
  M_180.setStopping(hold);

  right_motor.setVelocity(100.0, percent);
  left_motor.setVelocity (100.0, percent);
  four_bar.setMaxTorque  (100.0, percent);
  M_180.setVelocity      (100.0, percent);
  Kamasha.setVelocity    (100.0, percent);

  // calSlow gives more accurate gyro calibration at the cost of a longer wait
  Gyro2.calibrate(calSlow);
  while (Gyro2.isCalibrating()) {
    wait(20, msec);  // let the gyro finish calibrating
  }
  Gyro2.setRotation(0.0, degrees);  // zero the heading at start position

  TouchLED3.setColor(green);  // green = ready to run


  return 0;
}

// ── Autonomous Route ──────────────────────────────────────────────────────────
void route() {
  // your autonomous sequence goes here

}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  configureRobot();
  TouchLED3.pressed(route);  // run route() when the touch LED is pressed

  wait(15, msec);
}