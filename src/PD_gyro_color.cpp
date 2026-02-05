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
gyro Gyro8 = gyro(PORT8);
motor right_motor = motor(PORT1, true);
motor left_motor = motor(PORT7, false);
touchled C = touchled(PORT6);
touchled B = touchled(PORT12);
motor ArmMotorA = motor(PORT5, false);
motor ArmMotorB = motor(PORT11, true);
motor_group four_bar = motor_group(ArmMotorA, ArmMotorB);

colorsensor Color2 = colorsensor(PORT2);

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

// Include the IQ Library
#include "vex.h"

// Allows for easier use of the VEX Library
using namespace vex;


// Global Variables
bool autonomousRunning = false;

const float kAngleToleranceDeg = 1.0; // acceptable error band  // <- Modifiable

const float straightProportionalGain, straightDerivativeGain; 

const float pivotProportionalGain, pivotDerivativeGain;  

const float spinProportionalGain, spinDerivativeGain;  

const float currentSpeed, whiteReferenceValue, blackReferenceValue; 

const float speedKp, speedKd;


// Functions Prototypes
void pivotRightToAngle(float targetAngle);

void pivotLeftToAngle(float targetAngle);

void spinToAngle(float targetAngle);

void driveForwardUntilBlack(float driveSpeed, float targetAngle, float minimumDistanceDegrees);

void driveStraightWithRamp(float minimumSpeed, float targetDistanceDegrees, float targetAngle, float maximumSpeed);



// Defined functions
void pivotRightToAngle(float targetAngle)
{
  right_motor.stop(hold);
  
  float headingError = 0.0, previousHeadingError = 0.0;

  while (fabs(Gyro8.rotation() - targetAngle) > kAngleToleranceDeg)  
  {
    headingError = targetAngle - Gyro8.rotation();
    if (headingError > 180) headingError -= 360;
    if (headingError < -180) headingError += 360;

    double velocity = headingError * pivotProportionalGain + pivotDerivativeGain * (headingError - previousHeadingError);
    
    // Clamp velocity
    if (velocity > 100.0) velocity = 100.0;
    if (velocity < -100.0) velocity = -100.0;

    left_motor.setVelocity(fabs(velocity), percent);
    if (velocity > 0) {
      left_motor.spin(forward);
    } else {
      left_motor.spin(reverse);
    }
    previousHeadingError = headingError;
    wait(20, msec);
  }
  left_motor.stop(hold);
  right_motor.stop(hold);
}

void pivotLeftToAngle(float targetAngle)
{
  left_motor.stop(hold);

  float headingError = 0.0, previousHeadingError = 0.0;
  while (fabs(Gyro8.rotation() - targetAngle) > kAngleToleranceDeg)
  {
    headingError = targetAngle - Gyro8.rotation();
    if (headingError > 180) headingError -= 360;
    if (headingError < -180) headingError += 360;

    double velocity = headingError * pivotProportionalGain + pivotDerivativeGain * (headingError - previousHeadingError);
    
    // Clamp velocity
    if (velocity > 100.0) velocity = 100.0;
    if (velocity < -100.0) velocity = -100.0;

    right_motor.setVelocity(fabs(velocity), percent);
    if (velocity > 0) {
      right_motor.spin(forward);
    } else {
      right_motor.spin(reverse);
    }
    previousHeadingError = headingError;
    wait(20, msec);
  }
  right_motor.stop(hold);
  left_motor.stop(hold);
}

void spinToAngle(float targetAngle)
{
  float headingError = 0.0, previousHeadingError = 0.0;
  while (fabs(Gyro8.rotation() - targetAngle) > kAngleToleranceDeg)
  {
    headingError = targetAngle - Gyro8.rotation();

    // Normalize to shortest path (-180 to +180)
    if(headingError > 180.0) headingError -= 360.0;
    if(headingError < -180.0) headingError += 360.0;

    double velocity = headingError * spinProportionalGain + spinDerivativeGain * (headingError - previousHeadingError);
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

    previousHeadingError = headingError;
    wait(20, msec);
  }
  left_motor.stop(hold);
  right_motor.stop(hold);
}

// Add these helper functions after your global variables, before movement functions

float smoothstep(float x) {
  if (x <= 0.0) return 0.0;
  if (x >= 1.0) return 1.0;
  return x * x * (3.0 - 2.0 * x);
}

float clamp(float value, float min, float max) {
  return fmax(min, fmin(max, value));
}

// Then use them in your function:
void driveStraightWithVelocityPID(float minimumSpeed, float targetDistanceDegrees, float targetAngle, float maximumSpeed)
{
  
  float headingError = 0.0, previousHeadingError = 0.0;
  float speedError = 0.0, previousSpeedError = 0.0;
  

  
  right_motor.resetPosition();
  left_motor.resetPosition();
  
  while (true)
  {
    // Distance tracking
    float averageDistance = (fabs(right_motor.position(degrees)) + fabs(left_motor.position(degrees))) / 2.0;
    
    if (averageDistance >= targetDistanceDegrees) break;
    
    // Target speed from S-curve
    float progress = averageDistance / targetDistanceDegrees;
    float speedFactor;
    if (progress < 0.5) {
      speedFactor = smoothstep(progress * 2.0);
    } else {
      speedFactor = smoothstep((1.0 - progress) * 2.0);
    }
    float targetSpeed = minimumSpeed + (maximumSpeed - minimumSpeed) * speedFactor;
    
    // Actual speed from motor velocity sensor (built-in!)
    float actualSpeed = (right_motor.velocity(percent) + left_motor.velocity(percent)) / 2.0;
    
    // Speed PID
    speedError = targetSpeed - actualSpeed;
    float speedCorrection = speedError * speedKp + 
                            speedKd * (speedError - previousSpeedError);
    previousSpeedError = speedError;
    
    float baseSpeed = targetSpeed + speedCorrection;
    
    // Heading PID
    headingError = targetAngle - Gyro8.rotation();
    float headingCorrection = headingError * straightProportionalGain + 
                              straightDerivativeGain * (headingError - previousHeadingError);
    previousHeadingError = headingError;
    
    // Combine
    float rightVel = fmax(-100.0, fmin(100.0, baseSpeed + headingCorrection));
    float leftVel = fmax(-100.0, fmin(100.0, baseSpeed - headingCorrection));
    
    right_motor.setVelocity(fabs(rightVel), percent);
    left_motor.setVelocity(fabs(leftVel), percent);
    right_motor.spin(rightVel > 0 ? forward : reverse);
    left_motor.spin(leftVel > 0 ? forward : reverse);
    
    wait(20, msec);
  }
  
  right_motor.stop(hold);
  left_motor.stop(hold);
}
/*int configureRobot()
{
  C.setColor(red);
  B.setColor(red);

  right_motor.setStopping(hold);
  left_motor.setStopping(hold);

  straightProportionalGain = 0;   // <--- Modify
  straightDerivativeGain = 0;     // <--- Modify

  pivotProportionalGain = 0;      // <--- Modify
  pivotDerivativeGain = 0;        // <--- Modify

  spinProportionalGain = 0;       // <--- Modify
  spinDerivativeGain = 0;         // <--- Modify

  whiteReferenceValue = 0;        // <--- Modify
  blackReferenceValue = 0;        // <--- Modify
  
  Color2.setLight(30, percent);   // <--- Modify
  Gyro8.calibrate(calNormal);
  while (Gyro8.isCalibrating())
  {
    wait(20, msec); // let the gyro finish calibrating
  }
  Gyro8.setRotation(0.0, degrees);
  C.setColor(green);
  B.setColor(blue);
  return 0;
}
*/

int main()
{
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  


  wait(15, msec);
}
