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


// Robot configuration code.
gyro Gyro2 = gyro(PORT2);
motor right_motor = motor(PORT8, false);
motor left_motor = motor(PORT7, true);
touchled TouchLED3 = touchled(PORT3);
motor ArmMotorA = motor(PORT9, false);
motor ArmMotorB = motor(PORT10, false);
motor_group four_bar = motor_group(ArmMotorA, ArmMotorB);
motor M_180(PORT1,false);
motor Kamasha = motor(PORT4, false);

// generating and setting random seed
void initializeRandomSeed()
{
  wait(100, msec);
  double BrainTime = Brain.Timer.system() * 1000;
  double BrainVoltage = Brain.Battery.voltage();
  // Combine these values into a single integer
  int seed = int(
      BrainTime + BrainVoltage);
  // Set the seed
  srand(seed);
}

// Convert colorType to string
const char *convertColorToString(colorType col)
{
  if (col == colorType::red)
    return "red";
  else if (col == colorType::green)
    return "green";
  else if (col == colorType::blue)
    return "blue";
  else if (col == colorType::white)
    return "white";
  else if (col == colorType::yellow)
    return "yellow";
  else if (col == colorType::orange)
    return "orange";
  else if (col == colorType::purple)
    return "purple";
  else if (col == colorType::cyan)
    return "cyan";
  else if (col == colorType::black)
    return "black";
  else if (col == colorType::transparent)
    return "transparent";
  else if (col == colorType::red_violet)
    return "red_violet";
  else if (col == colorType::violet)
    return "violet";
  else if (col == colorType::blue_violet)
    return "blue_violet";
  else if (col == colorType::blue_green)
    return "blue_green";
  else if (col == colorType::yellow_green)
    return "yellow_green";
  else if (col == colorType::yellow_orange)
    return "yellow_orange";
  else if (col == colorType::red_orange)
    return "red_orange";
  else if (col == colorType::none)
    return "none";
  else
    return "unknown";
}

void vexcodeInit()
{

  // Initializing random seed.
  initializeRandomSeed();
}

#pragma endregion VEXcode Generated Robot Configuration


// Allows for easier use of the VEX Library
using namespace vex;


// Global Variables

float kAngleToleranceDeg = 1.0; // acceptable error band  // <- Modifiable

float Kp_straight = 1.5;

float Kd_straight = 4; 

float Kp_dist;

float Ki_dist;

float Kd_dist;

float Kp_PIVOT = 0.6;

float Kd_PIVOT = 4;  

float Kp_SPIN = 1.35;

float Kd_SPIN = 8.0;  


// Functions Prototypes
void pivotRightToAngle(float targetAngle);

void pivotLeftToAngle(float targetAngle);

void spinToAngle(float targetAngle);

void driveStraight(float velocity, float targetDistanceDegrees, float targetAngle, int settleTimeMs = 150, int timeoutMs = 3000);

float normalizeAngle(float angle) {
  if (angle >  180.0) angle -= 360.0;
  if (angle < -180.0) angle += 360.0;
  return angle;
}

// Defined functions
void pivotRightToAngle(float targetAngle)
{
  right_motor.setStopping(hold);
  left_motor.setStopping(hold);
  right_motor.stop();
  left_motor.stop();
  
  float headingError = 0.0;
  float previousHeadingError = 0.0;
  int settledCount = 0;

  previousHeadingError = normalizeAngle(targetAngle - Gyro2.rotation());

  
 
  while (true)  
  {
    headingError = normalizeAngle(targetAngle - Gyro2.rotation());

    
    double velocity = headingError * Kp_PIVOT + Kd_PIVOT * (headingError - previousHeadingError);
    
    // Clamp velocity
    if (velocity > 100.0) velocity = 100.0;
    if (velocity < -100.0) velocity = -100.0;

    left_motor.setVelocity(fabs(velocity), percent);
    if (velocity > 0) {
      left_motor.spin(reverse);
    } else {
      left_motor.spin(forward);
    }

    if (fabs(headingError) <= kAngleToleranceDeg) {
      settledCount++;
      if (settledCount > 5) {
        break;
      }
    } else {
      settledCount = 0;
    }
    previousHeadingError = headingError;
    wait(20, msec);
  }
  left_motor.stop();
  right_motor.stop();
}


void pivotLeftToAngle(float targetAngle)
{
  left_motor.setStopping(hold);
  right_motor.setStopping(hold);
  left_motor.stop();
  right_motor.stop();

  float headingError = 0.0, previousHeadingError = 0.0;
  int settledCount = 0;

  previousHeadingError = normalizeAngle(targetAngle - Gyro2.rotation());

  while (true)
  {
    headingError = normalizeAngle(targetAngle - Gyro2.rotation());


    double velocity = headingError * Kp_PIVOT + Kd_PIVOT * (headingError - previousHeadingError);
    
    // Clamp velocity
    if (velocity > 100.0) velocity = 100.0;
    if (velocity < -100.0) velocity = -100.0;

    right_motor.setVelocity(fabs(velocity), percent);
    if (velocity > 0) {
      right_motor.spin(forward);
    } else {
      right_motor.spin(reverse);
    }

    if (fabs(headingError) <= kAngleToleranceDeg) {
      settledCount++;
      if (settledCount > 5) {
        break;
      }
    } else {
      settledCount = 0;
    }
    previousHeadingError = headingError;
    wait(20, msec);
  }
  right_motor.stop();
  left_motor.stop();
}

void spinToAngle(float targetAngle)
{
  right_motor.setStopping(hold);
  left_motor.setStopping(hold);
  right_motor.stop();
  left_motor.stop();
  float headingError = 0.0, previousHeadingError = 0.0;
  int settledCount = 0;

  previousHeadingError = normalizeAngle(targetAngle - Gyro2.rotation());
 
  while (true)
  {
    headingError = normalizeAngle(targetAngle - Gyro2.rotation());


    
    double velocity = headingError * Kp_SPIN + Kd_SPIN * (headingError - previousHeadingError);
    // Clamp velocity
    if (velocity > 100.0) velocity = 100.0;
    if (velocity < -100.0) velocity = -100.0;

    left_motor.setVelocity(fabs(velocity), percent);
    right_motor.setVelocity(fabs(velocity), percent);

    if (velocity > 0) {
      right_motor.spin(forward);
      left_motor.spin(reverse);
    } else {
      right_motor.spin(reverse);
      left_motor.spin(forward);
    }
    if (fabs(headingError) <= kAngleToleranceDeg) {
      settledCount++;
      if (settledCount > 5) {
        break;
      }
    } else {
      settledCount = 0;
    }
    previousHeadingError = headingError;
    wait(20, msec);
  }
  left_motor.stop(hold);
  right_motor.stop(hold);
}

void driveStraight(float velocity, float targetDistanceDegrees, float targetAngle,
                   int settleTimeMs = 150, int timeoutMs = 3000)
{
  right_motor.setStopping(hold);
  left_motor.setStopping(hold);
  right_motor.resetPosition();
  left_motor.resetPosition();

  // Direction from velocity, distance always positive
  float directionSign = (velocity >= 0) ? 1.0 : -1.0;
  float actualTarget  = directionSign * fabs(targetDistanceDegrees);
  float MAX_DRIVE     = fabs(velocity);

  float distError     = 0.0;
  float prevDistError = 0.0;
  float distIntegral  = 0.0;

  float headingError     = 0.0;
  float prevHeadingError = normalizeAngle(targetAngle - Gyro2.rotation());

  const float SETTLE_THRESHOLD = 2.0;
  const float INTEGRAL_LIMIT   = 25.0;

  int settleTimer  = 0;
  int elapsedTimer = 0;

  while (true)
  {
    float averageDistance = (right_motor.position(degrees) + left_motor.position(degrees)) / 2.0;

    // Distance PID
    distError     = actualTarget - averageDistance;
    distIntegral += distError;
    if (distIntegral >  INTEGRAL_LIMIT) distIntegral =  INTEGRAL_LIMIT;
    if (distIntegral < -INTEGRAL_LIMIT) distIntegral = -INTEGRAL_LIMIT;

    float distOutput = Kp_dist * distError
                     + Ki_dist * distIntegral
                     + Kd_dist * (distError - prevDistError);

    if (distOutput >  MAX_DRIVE) distOutput =  MAX_DRIVE;
    if (distOutput < -MAX_DRIVE) distOutput = -MAX_DRIVE;

    prevDistError = distError;

    // Heading PD
    headingError = normalizeAngle(targetAngle - Gyro2.rotation());

    float headingCorrection = Kp_straight * headingError
                            + Kd_straight * (headingError - prevHeadingError);

    prevHeadingError = headingError;

    // Recalculate every loop — flips correctly during overshoot
    float correctionSign = (distOutput >= 0) ? 1.0 : -1.0;

    // Combine
    float rightVel = distOutput + headingCorrection * correctionSign;
    float leftVel  = distOutput - headingCorrection * correctionSign;

    if (rightVel >  100.0) rightVel =  100.0;
    if (rightVel < -100.0) rightVel = -100.0;
    if (leftVel  >  100.0) leftVel  =  100.0;
    if (leftVel  < -100.0) leftVel  = -100.0;

    right_motor.setVelocity(fabs(rightVel), percent);
    left_motor.setVelocity(fabs(leftVel),  percent);
    right_motor.spin(rightVel > 0 ? forward : reverse);
    left_motor.spin (leftVel  > 0 ? forward : reverse);

    // Settle check
    if (fabs(distError) < SETTLE_THRESHOLD) {
      settleTimer += 20;
      if (settleTimer >= settleTimeMs) break;
    } else {
      settleTimer = 0;
    }

    // Timeout check
    elapsedTimer += 20;
    if (elapsedTimer >= timeoutMs) break;

    wait(20, msec);
  }

  right_motor.stop(hold);
  left_motor.stop(hold);
}
int configureRobot()
{

  TouchLED3.setColor(red);
  right_motor.setStopping(hold);
  left_motor.setStopping(hold);
  four_bar.setStopping(hold);
  Kamasha.setStopping(hold);
  M_180.setStopping(hold);
  right_motor.setVelocity(100.0, percent);
  left_motor.setVelocity(100.0, percent);
  four_bar.setMaxTorque(100.0, percent);
  M_180.setVelocity(100.0, percent);
  Kamasha.setVelocity(100.0, percent);

  right_motor.setStopping(hold);
  left_motor.setStopping(hold);

  Gyro2.calibrate(calSlow);
  while (Gyro2.isCalibrating())
  {
    wait(20, msec); // let the gyro finish calibrating
  }
  Gyro2.setRotation(0.0, degrees);
  TouchLED3.setColor(green);

  return 0;
}

void route(){






}

int main()
{
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  configureRobot();
  TouchLED3.pressed(route);

  wait(15, msec);
}
