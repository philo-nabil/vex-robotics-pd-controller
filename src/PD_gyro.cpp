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
// touchled C = touchled(PORT6);
// touchled B = touchled(PORT12);
motor ArmMotorA = motor(PORT9, false);
motor ArmMotorB = motor(PORT10, false);
motor_group four_bar = motor_group(ArmMotorA, ArmMotorB);
motor M_180(PORT1,false);

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

const float Kp_straight = 1.5;

const float Kd_straight = 4; 

const float Kp_PIVOT = 0.6;

const float Kd_PIVOT = 4;  

const float Kp_SPIN = 1.35;

const float Kd_SPIN = 8.0;  

const float currentSpeed; 

const float speedKp, speedKd;


// Functions Prototypes
void pivotRightToAngle(float targetAngle);

void pivotLeftToAngle(float targetAngle);

void spinToAngle(float targetAngle);

void driveStraightWithRamp(float minimumSpeed, float targetDistanceDegrees, float targetAngle, float maximumSpeed);



// Defined functions
void pivotRightToAngle(float targetAngle)
{
  right_motor.stop(hold);
  
  float headingError = 0.0, previousHeadingError = 0.0;

  while (true)  
  {
    headingError = targetAngle - Gyro2.rotation();
    if (headingError > 180) headingError -= 360;
    if (headingError < -180) headingError += 360;
    if (fabs(headingError) <= kAngleToleranceDeg){
      break;
    }
    double velocity = headingError * Kp_PIVOT + Kd_PIVOT * (headingError - previousHeadingError);
    
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
  while (fabs(Gyro2.rotation() - targetAngle) > kAngleToleranceDeg)
  {
    headingError = targetAngle - Gyro2.rotation();
    if (headingError > 180) headingError -= 360;
    if (headingError < -180) headingError += 360;

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
    previousHeadingError = headingError;
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
  float headingError = 0.0, previousHeadingError = 0.0;
  while (true)
  {
    headingError = targetAngle - Gyro2.rotation();

    // Normalize to shortest path (-180 to +180)
    if(headingError > 180.0) headingError -= 360.0;
    if(headingError < -180.0) headingError += 360.0;
    if(fabs(headingError)<= kAngleToleranceDeg){
      break;
    }
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

    previousHeadingError = headingError;
    wait(20, msec);
  }
  left_motor.stop(hold);
  right_motor.stop(hold);
}

void driveStraightWithRamp(float minimumSpeed, float targetDistanceDegrees, float targetAngle, float maximumSpeed)
{
  float currentSpeed = minimumSpeed;
  float headingError = 0.0, previousHeadingError = 0.0;
  
  right_motor.resetPosition();
  left_motor.resetPosition();
  
  while (true)
  {
    float averageDistance = (fabs(right_motor.position(degrees)) + fabs(left_motor.position(degrees))) / 2.0;
    
    // exit condition
    if (averageDistance >= targetDistanceDegrees){
      break;
    } 
    
    // Simple linear ramp
    float progress = averageDistance / targetDistanceDegrees;
    
    if (progress < 0.5) {
      // Accelerate (first half)
      currentSpeed = minimumSpeed + (maximumSpeed - minimumSpeed) * (progress * 2.0);
    } else {
      // Decelerate (second half)
      currentSpeed = minimumSpeed + (maximumSpeed - minimumSpeed) * ((1.0 - progress) * 2.0);
    }
    
    // Heading PD
    headingError = targetAngle - Gyro2.rotation();
    if (headingError > 180.0) headingError -= 360.0;
    if (headingError < -180.0) headingError += 360.0;
    
    float headingCorrection = headingError * Kp_straight + Kd_straight * (headingError - previousHeadingError);
    previousHeadingError = headingError;
    
    // Apply to motors
    float rightVel = currentSpeed + headingCorrection;
    float leftVel = currentSpeed - headingCorrection;
    
    // Clamp
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


int configureRobot()
{

  right_motor.setStopping(hold);
  left_motor.setStopping(hold);

  Gyro2.calibrate(calSlow);
  while (Gyro2.isCalibrating())
  {
    wait(20, msec); // let the gyro finish calibrating
  }
  Gyro2.setRotation(0.0, degrees);
  return 0;
}


int main()
{
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  configureRobot();


  wait(15, msec);
}
