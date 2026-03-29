# VEX IQ Robotics — PD Controller

Hey! I'm Philo, and this is the code I built for **M&P Robotix**.

This is a **PD controller** — a simpler, cleaner version of the well-known PID controller. If you're just getting started with control systems, this is a great place to begin.

---

## Before You Begin

There are two things you need to do **before touching any code**:

### 1. Identify the Front of Your Robot
Before anything else, establish which side is the **face (front)** of your robot. This determines how you label and wire your **left** and **right** motors — and getting this wrong will flip all your turns.

### 2. Verify Motor Directions
Run both motors **forward** and confirm they spin in the correct direction. If a motor runs backwards, reverse it in software or physically re-wire it. This step saves you hours of confusing calibration later.

### 3. Calibrate the PD
Once your motors are confirmed, you're ready to tune `Kp` and `Kd`.

---

## What Is a PD Controller?

A **PD (Proportional-Derivative)** controller is a feedback control loop that drives a robot to a target heading or position:

```
velocity = (error × Kp) + ((error - previousError) × Kd)
```

- **Proportional (Kp)** — Reacts to how far off you are right now
- **Derivative (Kd)** — Predicts the trend, dampening overshoot and oscillation

No integral term means less complexity and no integral windup — perfect for VEX IQ autonomous routines.

---

## Features

- **PD Heading Control** — Smooth correction using gyroscope feedback
- **Multiple Turn Modes**:
  - Pivot turns (one wheel stationary)
  - Spin turns (both wheels counter-rotating)
- **Straight Driving** — Constant-speed drive with live heading correction
- **Robust Operation** — Velocity clamping and angle normalization built in
- **TouchLED Status Indicator** — Red during calibration, green when ready

---

## Hardware Configuration

| Component              | Port    |
|------------------------|---------|
| VEX IQ Brain (1st gen) | —       |
| Right Motor            | PORT 8  |
| Left Motor             | PORT 7  |
| Arm Motor A            | PORT 9  |
| Arm Motor B            | PORT 10 |
| M_180 Motor            | PORT 1  |
| Kamasha Motor          | PORT 4  |
| Gyroscope              | PORT 2  |
| TouchLED               | PORT 3  |

> The **Left Motor** is initialized with `true` (reversed) to account for its mirrored mounting orientation on the drive base.

---

## Core Functions

```cpp
// Pivot around the stationary right wheel to a target heading
void pivotRightToAngle(float targetAngle);

// Pivot around the stationary left wheel to a target heading
void pivotLeftToAngle(float targetAngle);

// Spin in place (point turn) to a target heading
void spinToAngle(float targetAngle);

// Drive straight at a fixed speed with PD heading correction
void driveStraight(
    float speed,                  // Drive speed (percent)
    float targetDistanceDegrees,  // Distance to travel (motor degrees)
    float targetAngle             // Heading to maintain (gyro degrees)
);
```

---

## How `driveStraight` Works

The function drives both motors at a base speed while using PD control to keep the heading locked:

```
rightVel = speed + headingCorrection
leftVel  = speed - headingCorrection
```

If the robot drifts right, `headingCorrection` speeds up the right motor and slows the left — and vice versa. The loop runs at **50 Hz (20 ms)** and exits once the robot has traveled `targetDistanceDegrees` for 5 consecutive cycles.

---

## Implementation Details

### Angle Normalization
All angular errors are wrapped to **[-180°, +180°]** so the robot always takes the shortest rotational path.

### Settled Count
Every function uses a `settledCount` — the robot must be within the tolerance band for **5 consecutive loops** before stopping. This prevents false exits from noise or a single lucky reading.

### Motor Settings
- Velocities clamped to **[-100, +100]%**
- All motors use **`hold`** braking mode for precise stopping
- Gyroscope uses **slow calibration** (`calSlow`) at startup and resets to 0°

### Robot Startup Sequence
1. TouchLED turns **red** — calibration in progress
2. Gyroscope calibrates (`calSlow`)
3. Gyroscope resets to 0°
4. TouchLED turns **green** — robot is ready
5. Press the TouchLED to run your autonomous route

---

## Getting Started

1. Open **VEXcode IQ**
2. Create a new project
3. Paste in the provided source code
4. Identify your robot's front and verify motor directions (see above)
5. Write your autonomous routine inside the `route()` function
6. Download and run — wait for the **green light**, then press the TouchLED

---

## Key Learnings

Building this project gave hands-on experience with:

- Implementing PD control algorithms for real-time robotics
- Gyroscope integration, slow calibration, and angle normalization
- Differential drive kinematics and motor coordination
- Gain tuning (`Kp`, `Kd`) for stable, responsive behavior
- Settled-count debouncing for reliable loop exits
- Embedded systems programming on the VEXcode platform

---

## Contributing

This is a learning project, but improvements are always welcome. Feel free to open an issue or submit a pull request.

---

## Contact

**Philobatir Nabil** — philobatir.nabil@proton.me

---
