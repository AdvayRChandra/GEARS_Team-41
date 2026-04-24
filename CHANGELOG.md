# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.2.0] - 2026-04-24

### Added
- `UltrasonicSensorState` dataclass in `modules/state.py` holding `distance`, `local_position`, `local_orientation`, `world_position`, and `world_orientation` per sensor
- `SensorInput.setup()` in `modules/sensors.py` primes `State` with an initial sensor reading before the update loop starts
- `SensorInput.get_distance_left()`, `get_distance_right()`, `get_distance_center()` replace the single `get_distance()` method; each writes directly to the corresponding `UltrasonicSensorState`
- `Location._euler_from_matrix()` in `modules/navigation_system.py` extracts `[yaw, pitch, roll]` from a ZYX rotation matrix with gimbal-lock handling
- `Location._compute_sensor_poses()` computes world-frame position and orientation for each ultrasonic sensor each tick using the robot's current pose and mount transforms stored in `SensorState`
- `Location.setup()` snapshots current motor positions as the odometry baseline; replaces implicit zero-initialization
- `Location.run_location_update()` continuous async loop that drives `update()` at a configurable interval using a wall-clock `dt`
- `Navigation` class in `modules/navigation_system.py` replaces the former thin update-loop wrapper; provides waypoint navigation (`go_to()`), world-frame obstacle projection (`get_obstacle_positions()`), timestamped pose logging, `setup()`, and `run_navigation_update()`
- `MotionController.setup()` async method in `modules/motion.py` syncs motor state once before the update loop
- `MotionController.run_motor_update()` continuous async loop that keeps `state.motor_left/right.position` and `is_moving` fresh between drive commands, preventing stale odometry during coasting
- `MotionController.speed` instance attribute set from `kwargs.get("speed", 50)` as the default speed for all movement commands
- `__main__` entry point in `modules/navigation_system.py` demonstrating concurrent async task setup with `SensorInput`, `Location`, `MotionController`, and `Navigation`

### Changed
- `SensorState` in `modules/state.py` replaces `ultrasonic_distance` (scalar), `lf_left_value`, `lf_right_value`, and `color_sensor_value` with three `UltrasonicSensorState` sub-objects (`ultrasonic_left`, `ultrasonic_right`, `ultrasonic_center`); removes `acceleration_raw`
- `SensorInput.__init__()` in `modules/sensors.py` now requires `state: State` as an explicit argument; replaces single ultrasonic sensor with three sensors on configurable pins (`ultrasonic_left_pin` default 16, `ultrasonic_right_pin` default 5, `ultrasonic_center_pin` default 26); removes `LineFinder` and `ColorSensor` support
- `SensorInput.get_gyro()` and `get_mag()` now write directly to `state.sensors` on each call in addition to returning the value
- `SensorInput.has_ultrasonic()` updated to return `True` only when all three ultrasonic sensors are initialized
- `SensorInput.update_state()` simplified: removed accelerometer, line finder, and color sensor branches; writes to the three `UltrasonicSensorState` sub-objects
- `Location.update_position()` in `modules/navigation_system.py` now calls `_compute_sensor_poses()` after computing the robot pose each tick
- `MotionController._set_moving()` replaced by `_drive()` which issues `start()` on both motors, sets `is_moving`, and syncs state in one call
- `MotionController.forward()`, `backward()`, `turn_left()`, `turn_right()` now accept `speed: int = None` and fall back to `self.speed` instead of a hardcoded default of 50

### Removed
- `SensorInput.get_accel()` — accelerometer data is no longer read or stored
- `SensorInput.get_distance()` — replaced by `get_distance_left/right/center()`
- `SensorInput.get_color()`, `is_black()`, `has_color_sensor()` — `ColorSensor` support removed
- `SensorInput.get_hall_value()`, `has_hall()` — deprecated Hall sensor stubs removed
- `SensorInput.get_line_left()`, `get_line_right()`, `has_line_finders()` — `LineFinder` support removed
- `MotionController._set_moving()` — replaced by `_drive()`

## [0.1.0] - 2026-04-24

### Added
- CONTRIBUTING.md with code standards based on ENGR 141/142 Code Standard (V4.1.3) and PEP 8 supplement for modules and classes
- `SensorState` dataclass in `modules/state.py` grouping all raw hardware sensor fields (IMU, ultrasonic, line finders, button, color)
- `NavigationState` dataclass in `modules/state.py` grouping computed pose fields (position, velocity, acceleration, orientation, angular quantities)
- `MotorState` dataclass in `modules/state.py` with `position` and `is_moving` fields per motor

### Changed
- Updated README.md with new header
- Updated README.md with link to Contributing Guidelines
- Created config.toml
- `State` in `modules/state.py` refactored from a flat dataclass into a container with `sensors: SensorState`, `nav: NavigationState`, `motor_left: MotorState`, and `motor_right: MotorState` sub-objects
- `SensorInput.update_state()` in `modules/sensors.py` updated to write to `state.sensors.*` fields
- `Location` in `modules/navigation_system.py` refactored to accept `state: State` directly; reads gyro from `state.sensors.angular_velocity_raw`, reads motor deltas from `state.motor_left/right.position`, and writes all pose output to `state.nav.*`; fixed missing `await` on `rotate_vector` call
- `MotionController` in `modules/motion.py` updated to sync `state.motor_left/right.position` and `is_moving` via `_sync_state()`; movement commands (`forward`, `backward`, `turn_left`, `turn_right`, `stop`) now set `is_moving` directly on state in addition to commanding the hardware

## [0.0.3] - 2026-02-10

### Added
- Build HAT modules package (`buildhat/`)
  - ColorSensor module for color sensing
  - Hat module for HAT interface
  - Motor and MotorPair modules for motor control
  - PassiveMotor module for passive motor sensing
  - BuildHAT serial interface module
  - Exception handling module

### Changed
- Updated README.md with credits for external code (buildhat and basehat modules)

## [0.0.2] - 2026-02-10

### Added
- Initial project setup with MIT License
- README.md file
- License file with copyright information for Team 41 members
- Grove Base HAT sensor modules package (`basehat/`)
  - Button module for button input handling
  - IMUSensor module for inertial measurement unit sensor
  - UltrasonicSensor module for distance sensing
  - LineFinder module for line detection
  - Proof of concept examples in `basehat/poc/`

### Changed
- Updated copyright information in LICENSE to include all team members:
  - Advay R. Chandra
  - Sarim Khan
  - Emma C. Wolcott
  - Katherine E. Hughley

## [0.0.1] - 2026-02-09

### Added
- Initial repository creation
- MIT License
- Basic project structure
