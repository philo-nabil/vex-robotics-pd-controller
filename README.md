# VEX IQ Robotics - PD Controller

A proportional-derivative (PD) controller implementation for VEX IQ robotics platform, featuring autonomous navigation with gyroscope-based heading correction.

## 🤖 Features

- **PD Control Algorithm**: Smooth, responsive robot movement with heading correction
- **Multiple Motion Modes**:
  - Pivot turns (single wheel rotation)
  - Spin turns (both wheels counter-rotating)
  - Straight driving with speed ramping
- **Safety Features**: Timeout protection, velocity clamping, sensor error handling

## 🛠️ Technical Details

### Hardware
- VEX IQ Brain (1st gen)
- 2x Drive Motors 
- 2x Arm Motors 
- Gyroscope Sensor 
- 2x Touch LED buttons

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
```

## 🚀 Getting Started

1. Open VEXcode IQ
2. Create a new project
3. Replace the default code with `src/PD.cpp`
4. Download to your VEX IQ robot
5. Press Button B or C to start autonomous routines

## 🎯 What I Learned

- Implementing PD control algorithms for real-time robotics
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

Philobatir Nabil  - [philobatir.nabil@proton.me]
