# VEX IQ Robotics - PD Controller

A proportional-derivative (PD) controller implementation for VEX IQ robotics platform, featuring autonomous navigation with gyroscope-based heading correction and trapezoidal motion profiling.

## Features

- **PD Control Algorithm**: Smooth, responsive robot movement with heading correction
- **Multiple Motion Modes**:
  - Pivot turns (single wheel rotation)
  - Spin turns (both wheels counter-rotating)
  - Straight driving with trapezoidal speed ramping
- **Advanced Motion Control**: Configurable acceleration/deceleration profiles
- **Robust Control**: Velocity clamping and angle normalization for reliable operation

## Technical Details

### Hardware Configuration

- **VEX IQ Brain** (1st gen)
- **Drive System**:
  - Right Motor (PORT8)
  - Left Motor (PORT7)
- **Four-Bar Lift**:
  - Arm Motor A (PORT9)
  - Arm Motor B (PORT10)
- **Additional Motor**: M_180 (PORT1)
- **Sensors**:
  - Gyroscope (PORT2)

### Control System Parameters

- **Control Loop Frequency**: 50 Hz (20ms update period)
- **Angle Tolerance**: ±1.0 degree
- **PD Gains**:
  - Straight Drive: Kp = 1.5, Kd = 4.0
  - Pivot Turn: Kp = 0.6, Kd = 4.0
  - Spin Turn: Kp = 1.35, Kd = 8.0

## Core Functions

```cpp
// Motion control functions
void pivotRightToAngle(float targetAngle)
  // Pivots around stationary right wheel to reach target heading

void pivotLeftToAngle(float targetAngle)
  // Pivots around stationary left wheel to reach target heading

void spinToAngle(float targetAngle)
  // Spins in place (point turn) to reach target heading

void driveStraightWithRamp(float minimumSpeed, float targetDistanceDegrees, 
                           float targetAngle, float maximumSpeed)
  // Drives straight with trapezoidal velocity profile
  // Parameters:
  //   - minimumSpeed: Starting/ending speed
  //   - targetDistanceDegrees: Distance to travel (in motor degrees)
  //   - targetAngle: Heading to maintain (gyro degrees)
  //   - maximumSpeed: Peak speed at midpoint
```

## Motion Profiling

The `driveStraightWithRamp()` function implements a **trapezoidal velocity profile**:
- **First 50%**: Linear acceleration from minimum to maximum speed
- **Last 50%**: Linear deceleration from maximum to minimum speed
- **Heading Correction**: PD control maintains straight line throughout

## Getting Started

1. Open **VEXcode IQ**
2. Create a new project
3. Copy the provided code into your project
4. Customize the `main()` function with your autonomous routine
5. Download to your VEX IQ robot
6. Run your program!

## Implementation Details

### Angle Normalization
All angular errors are normalized to the range [-180°, +180°] to ensure the robot takes the shortest rotational path.

### PD Control
```
velocity = error × Kp + (error - previousError) × Kd
```
- **Proportional term**: Responds to current error
- **Derivative term**: Dampens oscillation and overshooting

### Motor Coordination
- All velocities are clamped to [-100, +100] percent
- Motors use `hold` braking mode for precise positioning
- Gyroscope calibrates at startup for accurate heading reference

## Key Learnings

This project provided hands-on experience with:

- Implementing PD control algorithms for real-time robotics applications
- Trapezoidal motion profiling for smooth acceleration and deceleration
- Gyroscope sensor integration and angle normalization techniques
- Motor coordination and differential drive kinematics
- Embedded systems programming with VEXcode platform
- Control system gain tuning for stable and responsive movement

## License

MIT License - feel free to use this code for learning!

## Contributing

This is a learning project, but suggestions and improvements are welcome. Feel free to open an issue or submit a pull request.

## Contact

Philobatir Nabil - philobatir.nabil@proton.me

---

**Note**: The code includes motor group definitions and color conversion utilities that can be extended for competition use.