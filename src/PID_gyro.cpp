#pragma region VEXcode Generated Robot Configuration
// Make sure all required headers are included.
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

#include "vex.h"

using namespace vex;

// Brain should be defined by default
brain Brain;

// START IQ MACROS
#define waitUntil(condition) \
  do                         \
  {                          \
    wait(5, msec);           \
  } while (!(condition))

#define repeat(iterations) \
  for (int iterator = 0; iterator < iterations; iterator++)
// END IQ MACROS

// Robot configuration code.
gyro Gyro2 = gyro(PORT2);
motor right_motor = motor(PORT8, false);
motor left_motor = motor(PORT7, false);
motor ArmMotorA = motor(PORT9, false);
motor ArmMotorB = motor(PORT10, false);
motor_group four_bar = motor_group(ArmMotorA, ArmMotorB);
motor M_180(PORT1, false);

// generating and setting random seed
void initializeRandomSeed()
{
  wait(100, msec);
  double BrainTime = Brain.Timer.system() * 1000;
  double BrainVoltage = Brain.Battery.voltage();
  int seed = int(BrainTime + BrainVoltage);
  srand(seed);
}

void vexcodeInit()
{
  initializeRandomSeed();
}

#pragma endregion VEXcode Generated Robot Configuration

// Include the IQ Library
#include "vex.h"

// Allows for easier use of the VEX Library
using namespace vex;

// ============================================================================
// CONTROL PARAMETERS
// ============================================================================
const float kAngleToleranceDeg = 1.0;

// Straight drive PID
const float Kp_straight = 1.5;
const float Ki_straight = 0.05;
const float Kd_straight = 4.0;
const float maxIntegral_straight = 15.0;

// Pivot turn PID
const float Kp_PIVOT = 0.6;
const float Ki_PIVOT = 0.02;
const float Kd_PIVOT = 4.0;
const float maxIntegral_PIVOT = 10.0;

// Spin turn PID
const float Kp_SPIN = 1.35;
const float Ki_SPIN = 0.03;
const float Kd_SPIN = 8.0;
const float maxIntegral_SPIN = 12.0;

// Derivative filter coefficient (0-1, higher = more smoothing)
const float derivativeFilterAlpha = 0.7;

// Max heading correction as percentage of current speed
const float maxCorrectionRatio = 0.25;

// Speed ramp profile (percentages of total distance)
const float accelPhase = 0.3;   // 30% acceleration
const float decelPhase = 0.3;   // 30% deceleration
                                 // 40% constant speed

// ============================================================================
// PID CONTROLLER STRUCT
// ============================================================================
struct PIDController
{
  float Kp, Ki, Kd;
  float integralError;
  float previousError;
  float filteredDerivative;
  float maxIntegral;
  double lastTime;

  void init(float p, float i, float d, float maxInt)
  {
    Kp = p;
    Ki = i;
    Kd = d;
    maxIntegral = maxInt;
    reset();
  }

  void reset()
  {
    integralError = 0.0;
    previousError = 0.0;
    filteredDerivative = 0.0;
    lastTime = Brain.Timer.time();
  }

  float compute(float error)
  {
    double currentTime = Brain.Timer.time();
    float dt = currentTime - lastTime;
    lastTime = currentTime;

    // Prevent division by zero or unrealistic dt
    if (dt < 0.001) dt = 0.02; // Default to 20ms

    // Integral with anti-windup
    integralError += error * dt;
    if (integralError > maxIntegral) integralError = maxIntegral;
    if (integralError < -maxIntegral) integralError = -maxIntegral;

    // Filtered derivative (low-pass filter to reduce noise)
    float rawDerivative = (error - previousError) / dt;
    filteredDerivative = derivativeFilterAlpha * filteredDerivative + 
                         (1.0 - derivativeFilterAlpha) * rawDerivative;
    previousError = error;

    return Kp * error + Ki * integralError + Kd * filteredDerivative;
  }
};

// ============================================================================
// FUNCTION PROTOTYPES
// ============================================================================
void pivotRightToAngle(float targetAngle);
void pivotLeftToAngle(float targetAngle);
void spinToAngle(float targetAngle);
void driveStraightWithRamp(float minimumSpeed, float targetDistanceDegrees, 
                           float targetAngle, float maximumSpeed);
float normalizeAngle(float angle);
int configureRobot();

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================
float normalizeAngle(float angle)
{
  while (angle > 180.0) angle -= 360.0;
  while (angle < -180.0) angle += 360.0;
  return angle;
}

// ============================================================================
// MOVEMENT FUNCTIONS
// ============================================================================

void pivotRightToAngle(float targetAngle)
{
  // Right wheel stationary, left wheel moves
  right_motor.stop(hold);

  PIDController pidController;
  pidController.init(Kp_PIVOT, Ki_PIVOT, Kd_PIVOT, maxIntegral_PIVOT);

  while (true)
  {
    float headingError = normalizeAngle(targetAngle - Gyro2.rotation());
    
    if (fabs(headingError) <= kAngleToleranceDeg)
    {
      break;
    }

    float velocity = pidController.compute(headingError);

    // Clamp velocity
    if (velocity > 100.0) velocity = 100.0;
    if (velocity < -100.0) velocity = -100.0;

    left_motor.setVelocity(fabs(velocity), percent);
    if (velocity > 0)
    {
      left_motor.spin(forward);
    }
    else
    {
      left_motor.spin(reverse);
    }

    wait(20, msec);
  }

  left_motor.stop(hold);
  right_motor.stop(hold);
}

void pivotLeftToAngle(float targetAngle)
{
  // Left wheel stationary, right wheel moves
  left_motor.stop(hold);

  PIDController pidController;
  pidController.init(Kp_PIVOT, Ki_PIVOT, Kd_PIVOT, maxIntegral_PIVOT);

  while (true)
  {
    float headingError = normalizeAngle(targetAngle - Gyro2.rotation());
    
    if (fabs(headingError) <= kAngleToleranceDeg)
    {
      break;
    }

    float velocity = pidController.compute(headingError);

    // Clamp velocity
    if (velocity > 100.0) velocity = 100.0;
    if (velocity < -100.0) velocity = -100.0;

    right_motor.setVelocity(fabs(velocity), percent);
    
    // CORRECTED DIRECTION: positive headingError needs CCW turn
    // Right motor reverse = CCW when left is stationary
    if (velocity > 0)
    {
      right_motor.spin(reverse);  // Counter-clockwise
    }
    else
    {
      right_motor.spin(forward);  // Clockwise
    }

    wait(20, msec);
  }

  right_motor.stop(hold);
  left_motor.stop(hold);
}

void spinToAngle(float targetAngle)
{
  right_motor.setStopping(hold);
  left_motor.setStopping(hold);
  right_motor.stop();
  left_motor.stop();

  PIDController pidController;
  pidController.init(Kp_SPIN, Ki_SPIN, Kd_SPIN, maxIntegral_SPIN);

  while (true)
  {
    float headingError = normalizeAngle(targetAngle - Gyro2.rotation());
    
    if (fabs(headingError) <= kAngleToleranceDeg)
    {
      break;
    }

    float velocity = pidController.compute(headingError);

    // Clamp velocity
    if (velocity > 100.0) velocity = 100.0;
    if (velocity < -100.0) velocity = -100.0;

    left_motor.setVelocity(fabs(velocity), percent);
    right_motor.setVelocity(fabs(velocity), percent);

    // CORRECTED DIRECTION: positive headingError needs CCW turn
    // Left forward + Right reverse = CCW
    if (velocity > 0)
    {
      left_motor.spin(forward);    // Counter-clockwise
      right_motor.spin(reverse);
    }
    else
    {
      left_motor.spin(reverse);    // Clockwise
      right_motor.spin(forward);
    }

    wait(20, msec);
  }

  left_motor.stop(hold);
  right_motor.stop(hold);
}

void driveStraightWithRamp(float minimumSpeed, float targetDistanceDegrees, 
                           float targetAngle, float maximumSpeed)
{
  float currentSpeed = minimumSpeed;
  
  right_motor.resetPosition();
  left_motor.resetPosition();

  PIDController headingController;
  headingController.init(Kp_straight, Ki_straight, Kd_straight, maxIntegral_straight);

  // Determine direction of travel
  int direction = (targetDistanceDegrees > 0) ? 1 : -1;
  float absTargetDistance = fabs(targetDistanceDegrees);

  // Calculate ramp distances
  float accelDistance = absTargetDistance * accelPhase;
  float decelStartDistance = absTargetDistance * (1.0 - decelPhase);

  while (true)
  {
    // Calculate signed average distance
    float rightPos = right_motor.position(degrees);
    float leftPos = left_motor.position(degrees);
    float averageDistance = (rightPos + leftPos) / 2.0;
    float absAverageDistance = fabs(averageDistance);

    // Exit condition
    if (absAverageDistance >= absTargetDistance)
    {
      break;
    }

    // Asymmetric speed ramp profile
    if (absAverageDistance < accelDistance)
    {
      // Acceleration phase
      float accelProgress = absAverageDistance / accelDistance;
      currentSpeed = minimumSpeed + (maximumSpeed - minimumSpeed) * accelProgress;
    }
    else if (absAverageDistance > decelStartDistance)
    {
      // Deceleration phase
      float remainingDistance = absTargetDistance - absAverageDistance;
      float decelDistance = absTargetDistance * decelPhase;
      float decelProgress = remainingDistance / decelDistance;
      currentSpeed = minimumSpeed + (maximumSpeed - minimumSpeed) * decelProgress;
    }
    else
    {
      // Constant speed phase
      currentSpeed = maximumSpeed;
    }

    // Heading correction with PID
    float headingError = normalizeAngle(targetAngle - Gyro2.rotation());
    float headingCorrection = headingController.compute(headingError);

    // Limit correction to prevent speed profile destruction
    float maxCorrection = currentSpeed * maxCorrectionRatio;
    if (headingCorrection > maxCorrection) headingCorrection = maxCorrection;
    if (headingCorrection < -maxCorrection) headingCorrection = -maxCorrection;

    // Apply to motors
    float rightVel = (currentSpeed + headingCorrection) * direction;
    float leftVel = (currentSpeed - headingCorrection) * direction;

    // Clamp to motor limits
    if (rightVel > 100.0) rightVel = 100.0;
    if (rightVel < -100.0) rightVel = -100.0;
    if (leftVel > 100.0) leftVel = 100.0;
    if (leftVel < -100.0) leftVel = -100.0;

    right_motor.setVelocity(fabs(rightVel), percent);
    left_motor.setVelocity(fabs(leftVel), percent);
    right_motor.spin(rightVel > 0 ? forward : reverse);
    left_motor.spin(leftVel > 0 ? forward : reverse);

    wait(20, msec);
  }

  right_motor.stop(hold);
  left_motor.stop(hold);
}

// ============================================================================
// CONFIGURATION
// ============================================================================
int configureRobot()
{
  right_motor.setStopping(hold);
  left_motor.setStopping(hold);

  Gyro2.calibrate(calSlow);
  while (Gyro2.isCalibrating())
  {
    wait(20, msec);
  }
  Gyro2.setRotation(0.0, degrees);
  
  return 0;
}

// ============================================================================
// MAIN
// ============================================================================
int main()
{
  vexcodeInit();
  configureRobot();

  // Example autonomous routine
  // driveStraightWithRamp(20, 720, 0, 80);   // Drive forward 2 rotations
  // spinToAngle(90);                          // Turn to 90 degrees
  // driveStraightWithRamp(20, 360, 90, 80);  // Drive forward 1 rotation at 90°

  wait(15, msec);
}