# VEX IQ Robotics - PD Controller

A proportional-derivative (PD) controller implementation for VEX IQ robotics platform, featuring autonomous navigation with gyroscope-based heading correction and color sensor-based line following.

## 🤖 Features

- **PD Control Algorithm**: Smooth, responsive robot movement with heading correction
- **Autonomous Navigation**: Line following using color sensor
- **Multiple Motion Modes**:
  - Pivot turns (single wheel rotation)
  - Spin turns (both wheels counter-rotating)
  - Straight driving with speed ramping
- **Safety Features**: Timeout protection, velocity clamping, sensor error handling

## 🛠️ Technical Details

### Hardware
- VEX IQ Brain
- 2x Drive Motors (Ports 1, 7)
- 2x Arm Motors (Ports 5, 11)
- Gyroscope Sensor (Port 8)
- Color Sensor (Port 2)
- 2x Touch LED buttons (Ports 6, 12)

### Control System
- **Control Loop Frequency**: 50 Hz (20ms period)
- **PD Gains**: Tuned for stable, responsive movement
- **Angle Tolerance**: ±1 degree

## 📋 Code Structure
```cpp
// Core motion functions
pivotRightToAngle()     // Pivot on right wheel
pivotLeftToAngle()      // Pivot on left wheel
spinToAngle()           // Spin in place
driveStraightWithRamp() // Drive with acceleration/deceleration
driveForwardUntilBlack() // Line following
```

## 🚀 Getting Started

1. Open VEXcode IQ
2. Create a new project
3. Replace the default code with `src/PD.cpp`
4. Download to your VEX IQ robot
5. Press Button B or C to start autonomous routines

## 🎯 What I Learned

- Implementing PD control algorithms for real-time robotics
- Sensor fusion (gyroscope + color sensor)
- Motor control and coordination
- Timeout protection and error handling
- Code optimization for embedded systems

## 📊 Improvements Over Original

See [IMPROVEMENTS.md](docs/IMPROVEMENTS.md) for detailed code review and fixes including:
- Proper derivative calculation with dt division
- Velocity clamping and bounds checking
- Timeout protection on all operations
- Refactored magic numbers to named constants
- Enhanced error handling

## 📝 License

MIT License - feel free to use this code for learning!

## 🤝 Contributing

This is a learning project, but suggestions are welcome! Feel free to open an issue.

## 📧 Contact

[Your Name] - [your-email@example.com]
Portfolio: [your-portfolio-site.com]