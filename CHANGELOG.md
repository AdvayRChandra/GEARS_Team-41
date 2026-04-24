# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

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
