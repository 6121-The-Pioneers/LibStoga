<div align="center">

<img width="1920" height="540" alt="LibStoga" src="https://github.com/user-attachments/assets/7c30bac4-922c-4bed-9c62-17c0cee776a9" />
<br><br>

**A fast, minimalist, open-source motion control library for VEX V5RC robots.**

Built on [PROS](https://pros.cs.purdue.edu/), LibStoga provides PID control, two odometry implementations (3-wheel and IMU-based), autonomous path execution, and the SPID adaptive controller — all in a clean, well-documented C++ API.

> Alpha release. Core functionality is implemented and competition-tested. Some features are still in progress (see status table below).

</div>

---

## Installation

Download the latest `.zip` from the [Releases page](https://github.com/6121-The-Pioneers/LibStoga/releases/tag/v0.0.1-alpha) and follow the PROS template installation process:

```bash
pros conduct fetch LibStoga@0.0.1-alpha
pros conduct apply LibStoga
```

Or manually copy the `include/LibStoga` and `src/LibStoga` directories into your PROS project.

**Requirements:** PROS 3.x, VEX V5 brain

---

## Overview

All classes live in the `ls` namespace. The library is organized into the following modules:

| Module | Header | Description |
|---|---|---|
| PID | `pid.h` | Standard PID controller |
| SmartPID | `spid.h` | Self-tuning adaptive PID (SPID algorithm) |
| Odometry | `odom.h` | 3-wheel and IMU-based position tracking |
| Chassis | `chassis.h` | Tank drive chassis with autonomous movement |
| Tracking | `tracking.h` | Tracking wheel wrappers (Rotation + ADI Encoder) |
| Geometry | `geometry.h` | `Angle` class and conversion utilities |
| Timer | `Timer.hpp` | Timing utilities for autonomous routines |

---

## PID — `pid.h`

Standard PID controller implementing the `AbstractPID` interface.

### `ls::PID`

```cpp
ls::PID(float kP, float kI, float kD, float windupRange, bool signFlipReset);
```

| Parameter | Description |
|---|---|
| `kP` | Proportional gain |
| `kI` | Integral gain |
| `kD` | Derivative gain |
| `windupRange` | Error range within which integral accumulates |
| `signFlipReset` | If true, resets integral when error sign flips |

#### Methods

```cpp
double update(const double error);
```
Calculates and returns the PID output given the current error. Call this every control loop iteration.

```cpp
void reset();
```
Resets integral accumulator and previous error. Does not reset gains.

#### Example

```cpp
ls::PID lateral(5.0, 0.1, 0.3, 10.0, true);

// In your control loop:
double output = lateral.update(target - current_position);
```

---

## SmartPID — `spid.h`

An adaptive extension of PID that uses gradient descent to self-tune kP, kI, and kD in real time. Implements the SPID algorithm. See the [SPID repository](https://github.com/rishitv26/SPID) for full algorithmic details.

> **Warning:** SmartPID can produce fast, aggressive output changes. Only modify `learning_constant` and `windup` if you understand their effect on your specific system.

### `ls::SmartPID`

```cpp
ls::SmartPID(std::function<double()> actual, double cc = 1, double w = 10,
             double lc = 0.001, double max = 127);
```

| Parameter | Default | Description |
|---|---|---|
| `actual` | — | Function returning the actual current motor output. Must return a value in `[-max, max]`. |
| `cc` | `1.0` | Correction constant — controls overall motion speed shape |
| `w` | `10.0` | Integral windup limit |
| `lc` | `0.001` | Learning rate — controls how fast gains adapt |
| `max` | `127.0` | Maximum allowed output value |

Alternatively, use the parameter struct:

```cpp
ls::smart_pid_parameters_t params = {
    .actual_output = []() { return motor.get_actual_velocity(); },
    .correction_constant = 1.0,
    .windup = 10.0,
    .learning_constant = 0.001,
    .max_value = 127.0
};
ls::SmartPID controller(params);
```

#### Methods

```cpp
double update(const double error);
void reset();
```
Same interface as `ls::PID`. `reset()` clears speed, integral, and derivative state — but not the learned gains.

---

## Odometry — `odom.h`

Two odometry implementations, both extending `AbstractOdom`. Position tracking uses a bearing coordinate system where (0, 0) and 0° correspond to the robot's starting pose.

### `ls::Position`

A compact (X, Y, θ) representation.

```cpp
ls::Position pos(0.0, 0.0, 0.0); // x, y, heading (degrees, bearing)

double dist = pos.distanceFromPoint(other);       // always positive
double signed_dist = pos.distanceFromPointSigned(other); // negative if behind
int dir = pos.isBehind(other);                    // -1 behind, 0 neutral, 1 in front
ls::Angle heading = pos.angleToPosition(other);   // bearing to other point
```

---

### `ls::ThreeWheelOdom`

Pure 3-wheel odometry using two parallel and one perpendicular tracking wheel. No IMU required.

```cpp
// Using parameter struct (recommended):
ls::threewheel_odom_parameters_t params = {
    .right  = { .port = 1, .radius = 2.75, .reversed = false },
    .center_to_right = 4.0,   // inches
    .left   = { .port = 2, .radius = 2.75, .reversed = true },
    .center_to_left = 4.0,
    .back   = { .port = 3, .radius = 2.75, .reversed = false },
    .center_to_back = 3.5
};
ls::ThreeWheelOdom odom(params);

// Or using port-based initialization:
ls::ThreeWheelOdom odom(4.0, 4.0, 3.5);
odom.initialize({1, 2, 3}); // right, left, back ports
```

#### Key Methods

```cpp
void compute();              // Call each iteration to update position
double getX();               // Current X in inches
double getY();               // Current Y in inches
double getAngle();           // Current heading in degrees (bearing)
ls::Position getPosition();  // Full position object

void resetX();
void resetY();
void resetAngle();
void resetAll();

double getDeltaX();          // Change in X since last call
double getDeltaY();          // Change in Y since last call
ls::Angle getDeltaAngle();   // Change in heading since last call
```

---

### `ls::ImuOdom`

2-wheel + IMU odometry. Uses one horizontal (perpendicular) and one vertical (parallel) tracking wheel alongside an inertial sensor for heading.

```cpp
// Using parameter struct (recommended):
ls::imu_odom_parameters_t params = {
    .parallel         = { .port = 1, .radius = 2.75, .reversed = false },
    .center_to_parallel = 4.0,
    .perpendicular    = { .port = 2, .radius = 2.75, .reversed = false },
    .center_to_perpendicular = 3.5,
    .imu_port = 10
};
ls::ImuOdom odom(params);

// Or using port-based initialization:
ls::ImuOdom odom(3.5, 4.0);
odom.initialize({2, 1, 10}); // horiz, vert, IMU ports
```

Same `compute()`, `getX/Y/Angle()`, `reset*()`, and `getDelta*()` interface as `ThreeWheelOdom`.

---

## Chassis — `chassis.h`

Wraps a tank drive robot with autonomous movement methods. Requires motor groups, an odometry object, and PID controllers for lateral and angular control.

### `ls::Chassis`

```cpp
ls::Chassis(
    pros::MotorGroup& right,
    pros::MotorGroup& left,
    ls::AbstractOdom& odom,
    ls::AbstractPID& lateral_control,
    ls::AbstractPID& angular_control,
    double turn_sensitivity  = 1.0,    // >1 prioritizes turning, <1 prioritizes lateral
    double threshold_lateral = 1.0,    // inches — exit threshold for movement
    double threshold_angular = 5.0,    // degrees — exit threshold for turning
    unsigned int threshold_timeout = 500  // ms to remain within threshold before exiting
);
```

#### Methods

```cpp
void moveToPoint(double X, double Y, unsigned int timeout, bool reverse = false);
```
Drives the robot to the given (X, Y) coordinate in the field using odometry feedback. `timeout` is in milliseconds. Set `reverse = true` to approach the point driving backwards.

```cpp
void turnToPoint(double X, double Y, unsigned int timeout, bool reverse = false);
```
Turns the robot to face a given (X, Y) coordinate.

#### Full Setup Example

```cpp
pros::MotorGroup right_motors({1, 2, 3});
pros::MotorGroup left_motors({-4, -5, -6});

ls::PID lateral(5.0, 0.05, 0.2, 10.0, true);
ls::PID angular(3.0, 0.0,  0.1, 5.0,  true);

ls::imu_odom_parameters_t odom_params = {
    .parallel              = {1, 2.75, false},
    .center_to_parallel    = 4.0,
    .perpendicular         = {2, 2.75, false},
    .center_to_perpendicular = 3.5,
    .imu_port              = 10
};
ls::ImuOdom odom(odom_params);

ls::Chassis chassis(right_motors, left_motors, odom, lateral, angular);

// In autonomous:
chassis.moveToPoint(24.0, 24.0, 3000);
chassis.turnToPoint(48.0, 0.0, 2000);
```

---

## Tracking Wheels — `tracking.h`

### `ls::TrackingWheel`

Wrapper around either `pros::Rotation` (smart port) or `pros::adi::Encoder` (ADI port).

```cpp
// Smart port (Rotation sensor):
ls::TrackingWheel wheel(1, 2.75, false); // port, radius (in), reversed

// ADI Encoder:
ls::TrackingWheel wheel('A', 'B', 1.375, false); // upper port, lower port, radius, reversed
```

#### Methods

```cpp
double getLinearSpeed();         // Linear speed in in/s
double getLinearDistance();      // Total displacement in inches
double getLinearDeltaDistance(); // Change in distance since last call
void setRadius(double radius);   // Update wheel radius
void reverse();                  // Flip direction
void reset();                    // Reset encoder position
```

---

## Geometry — `geometry.h`

### `ls::Angle`

A degree-based angle class with bearing convention.

```cpp
ls::Angle a(90.0);

double deg  = a.getAngle();           // 90.0
double rad  = a.convertToRadians();   // π/2
double norm = a.normalize();          // [0, 360)
ls::Angle diff = a.minimumAngleDifference(other); // signed diff in [-180, 180]

a = 45.0;   // assignment
a += other; // increment
```

### Utility Functions

```cpp
double ls::degreesToRadians(double degrees);
double ls::radiansToDegrees(double radians);
```

---

## Timer — `Timer.hpp`

Useful for autonomous timing and blocking waits.

```cpp
ls::Timer t(2000); // 2 second timer

t.getTimeSet();     // returns 2000
t.getTimePassed();  // ms elapsed
t.getTimeLeft();    // ms remaining
t.isDone();         // true when expired
t.waitUntilDone();  // blocking wait

t.pause();
t.resume();
t.reset();
t.set(1000);        // reassign to 1 second
```

---

## Status

| Feature | Status |
|---|---|
| PID controller | Complete |
| SmartPID (SPID) | In progress |
| 3-Wheel Odometry | Complete |
| IMU Odometry | Complete |
| Tracking Wheel wrappers | Complete |
| Geometry utilities | Complete |
| Timer | Complete |
| Chassis `moveToPoint` | In progress |
| Chassis `turnToPoint` | In progress |
| Swerve drive support | Planned |

---

## Contributing

Pull requests are welcome. The project follows PROS conventions. If you find a bug or want to request a feature, open an issue.

---
