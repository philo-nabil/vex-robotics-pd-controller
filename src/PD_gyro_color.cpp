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
motor_group Arm = motor_group(ArmMotorA, ArmMotorB);

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

const double kAngleToleranceDeg = 1.0; // acceptable error band

float headingError, previousHeadingError;

float straightProportionalGain, straightDerivativeGain;

float pivotProportionalGain, pivotDerivativeGain;

float spinProportionalGain, spinDerivativeGain;

float currentSpeed, whiteReferenceValue, blackReferenceValue;



// Functions Prototypes
void pivotRightToAngle(double targetAngle);

void pivotLeftToAngle(double targetAngle);

void spinToAngle(double targetAngle);

void driveForwardUntilBlack(double driveSpeed, double targetAngle, double minimumDistanceDegrees);

void driveStraightWithRamp(double minimumSpeed, double targetDistanceDegrees, double targetAngle, double maximumSpeed);



// Defined functions
void pivotRightToAngle(double targetAngle)
{
  previousHeadingError = 0.0;
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
}

void pivotLeftToAngle(double targetAngle)
{
  previousHeadingError = 0.0;
  while (fabs(Gyro8.rotation() - targetAngle) > kAngleToleranceDeg)
  {
    headingError = targetAngle - Gyro8.rotation();
    if (headingError > 180) headingError -= 360;
    if (headingError < -180) headingError += 360;

    // right_motor.setVelocity((headingError * pivotProportionalGain + pivotDerivativeGain * (headingError - previousHeadingError)), percent);
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
}

void spinToAngle(double targetAngle)
{
  previousHeadingError = 0.0;
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
      right_motor.spin(reverse);
      left_motor.spin(forward);
    } else {
      right_motor.spin(forward);
      left_motor.spin(reverse);
    }

    previousHeadingError = headingError;
    wait(20, msec);
  }
  left_motor.stop(hold);
  right_motor.stop(hold);
}

void driveForwardUntilBlack(double driveSpeed, double targetAngle, double minimumDistanceDegrees)
{
  previousHeadingError = 0.0;
  right_motor.setPosition(0.0, degrees);
  while (!(((static_cast<float>(Color2.brightness()) - blackReferenceValue) * 100.0) / (whiteReferenceValue - blackReferenceValue) < 80.0 && fabs(static_cast<float>(right_motor.position(degrees))) >= minimumDistanceDegrees))
  {
    headingError = targetAngle - Gyro8.rotation();
    right_motor.setVelocity((driveSpeed + (headingError * straightProportionalGain + straightDerivativeGain * (headingError - previousHeadingError))), percent);
    left_motor.setVelocity((driveSpeed - (headingError * straightProportionalGain + straightDerivativeGain * (headingError - previousHeadingError))), percent);
    right_motor.spin(forward);
    left_motor.spin(forward);
    previousHeadingError = headingError;
    wait(20, msec);
  }
  while (!(((static_cast<float>(Color2.brightness()) - blackReferenceValue) * 100.0) / (whiteReferenceValue - blackReferenceValue) < 40.0 && fabs(static_cast<float>(right_motor.position(degrees))) >= minimumDistanceDegrees))
  {
    headingError = targetAngle - Gyro8.rotation();
    right_motor.setVelocity((driveSpeed * 0.5 + (headingError * straightProportionalGain + straightDerivativeGain * (headingError - previousHeadingError))), percent);
    left_motor.setVelocity((driveSpeed * 0.5 - (headingError * straightProportionalGain + straightDerivativeGain * (headingError - previousHeadingError))), percent);
    right_motor.spin(forward);
    left_motor.spin(forward);
    previousHeadingError = headingError;
    wait(20, msec);
  }
  right_motor.stop(hold);
  left_motor.stop(hold);
}

void driveStraightWithRamp(double minimumSpeed, double targetDistanceDegrees, double targetAngle, double maximumSpeed)
{
  currentSpeed = minimumSpeed;
  previousHeadingError = 0.0;
  right_motor.setPosition(0.0, degrees);
  while (!(fabs(static_cast<float>(right_motor.position(degrees))) >= targetDistanceDegrees * 0.5))
  {
    headingError = targetAngle - Gyro8.rotation();
    right_motor.setVelocity((currentSpeed + (headingError * straightProportionalGain + straightDerivativeGain * (headingError - previousHeadingError))), percent);
    left_motor.setVelocity((currentSpeed - (headingError * straightProportionalGain + straightDerivativeGain * (headingError - previousHeadingError))), percent);
    right_motor.spin(forward);
    left_motor.spin(forward);
    previousHeadingError = headingError;
    if (fabs(static_cast<float>(currentSpeed)) < fabs(static_cast<float>(maximumSpeed)))
    {
      currentSpeed = currentSpeed * 1.3;
    }
    wait(20, msec);
  }
  while (!(fabs(static_cast<float>(right_motor.position(degrees))) >= targetDistanceDegrees))
  {
    if (fabs(static_cast<float>(currentSpeed)) > fabs(static_cast<float>(minimumSpeed)))
    {
      currentSpeed = currentSpeed * 0.875;
    }
    headingError = targetAngle - Gyro8.rotation();
    right_motor.setVelocity((currentSpeed + (headingError * straightProportionalGain + straightDerivativeGain * (headingError - previousHeadingError))), percent);
    left_motor.setVelocity((currentSpeed - (headingError * straightProportionalGain + straightDerivativeGain * (headingError - previousHeadingError))), percent);
    right_motor.spin(forward);
    left_motor.spin(forward);
    previousHeadingError = headingError;
    wait(20, msec);
  }
  right_motor.stop(hold);
  left_motor.stop(hold);

}

// "when started" hat block
int configureRobot()
{
  C.setColor(red);
  B.setColor(red);

  right_motor.setStopping(hold);
  left_motor.setStopping(hold);
  Arm.setMaxTorque(100.0, percent);

  straightProportionalGain = -1.825;
  straightDerivativeGain = -5.6;

  pivotProportionalGain = -3.3;
  pivotDerivativeGain = -1.4;

  spinProportionalGain = 3.0;
  spinDerivativeGain = 9.5;

  whiteReferenceValue = 89.0;
  blackReferenceValue = 12.0;
  
  Color2.setLight(30, percent);
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


void B_handleButtonPressed()
{
  driveForwardUntilBlack(-60.0, 0.0, 220.0);
  pivotRightToAngle(-90.0);
  driveStraightWithRamp(20.0, 200.0, -90.0, 70.0);
  spinToAngle(-89.0);
  Arm.spinFor(forward, 40.0, degrees, true);
  driveStraightWithRamp(-30.0, 330.0, -89.0, -70.0);
}

// "when C pressed" hat block
void C_handleButtonPressed()
{
  driveForwardUntilBlack(60.0, 0.0, 0.0);
  pivotRightToAngle(-90.0);
  driveStraightWithRamp(20.0, 200.0, -90.0, 70.0);
  spinToAngle(-89.0);
  Arm.spinFor(forward, 40.0, degrees, true);
  driveStraightWithRamp(-30.0, 330.0, -89.0, -70.0);
}


int main()
{
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  configureRobot();

  // register event handlers
  B.pressed(B_handleButtonPressed);
  C.pressed(C_handleButtonPressed);

  wait(15, msec);
}
