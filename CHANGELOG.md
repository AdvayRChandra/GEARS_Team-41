# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [1.0.0] - 2026-04-25

### Added
- `MapConfig` dataclass in `modules/config.py` — static configuration for the 2D occupancy grid (`map_width`, `map_height`, `resolution`, `magnetic_threshold`, `ir_threshold`, `origin`, `team`, `unit_length`, `unit`)
- `RobotConfig.map` field (`MapConfig`) in `modules/config.py`
- `Map` class in `modules/navigation_system.py` — 2D occupancy grid updated each navigation tick; stores the grid as a NumPy array and exposes `_world_to_grid()`, `update_path()`, `update_obstacles()`, and `save_map()` methods
- `Map.get_obstacle_states()` in `modules/navigation_system.py` — returns raw grid cell values for the 4 cells adjacent to the robot in its current discrete heading as `[front, left, back, right]`; -1 indicates out-of-bounds
- `Navigation.map` (`Map`) attribute and wiring into `run_navigation_update()` so the occupancy grid is updated every tick
- `IMUSensorConfig` dataclass in `modules/config.py` — mount transform from IMU frame to robot frame (position and orientation)
- `SensorConfig.imu` field (`IMUSensorConfig`) in `modules/config.py` with default zero offset
- `SensorState.mag_world_position` (`np.ndarray`) in `modules/state.py` — world-frame position of the magnetometer, updated each tick
- `IRSensorState.value` field replaces the former `value1`/`value2` fields; each element now has its own `IRSensorState` instance with an independent `world_position`
- `SensorConfig.ir_sensor_left` and `SensorConfig.ir_sensor_right` (`IRSensorConfig`) in `modules/config.py` — each element has its own `pin` and `local_position`; `ir_sensor_right` defaults to pin 3
- `SensorState.ir_sensor_left` and `SensorState.ir_sensor_right` (`IRSensorState`) in `modules/state.py`
- `Navigation`, `Map`, `IMUSensorConfig`, `MapConfig`, `UltrasonicSensorState`, `IRSensorState` added to `modules/__init__.py` exports
- `NavigationState.discrete_orientation` (`list [dx, dy]`) in `modules/state.py` — robot heading snapped to the nearest cardinal direction; updated each tick after gyro integration
- `NavigationState.obstacle_neighbors` (`list [front, left, back, right]`) in `modules/state.py` — raw grid cell values of the four cells adjacent to the robot in its current discrete heading; -1 indicates out-of-bounds
- `Location._DISCRETE_DIRS` class-level constant and `_compute_discrete_orientation()` method in `modules/navigation_system.py` — snaps continuous yaw to one of four cardinal directions ([1,0], [0,1], [-1,0], [0,-1])
- `Navigation.determine_turn()` in `modules/navigation_system.py` — inspects `obstacle_neighbors` and returns a direction code (0=forward, 1=right, -1=left, 2=back) using priority-based cell-value logic (free space first, then previously traveled path; front > right > left > back within each tier)
- `Navigation.automatic_turn()` in `modules/navigation_system.py` — calls `determine_turn()` and executes the corresponding `turn_degrees()` call, or drives forward if no turn is needed
- `Navigation.turn_degrees()` in `modules/navigation_system.py` — turns the robot by a given number of degrees using `turn_left`/`turn_right` and stops within `angle_tolerance`
- `RobotConfig.initial_heading` (`float`) in `modules/config.py` — explicit starting yaw in degrees, applied to the yaw component of `initial_orientation` at startup (default: 0.0)
- `IMUSensorConfig.local_orientation` (`np.ndarray`) field — rotation from IMU frame to robot frame as [yaw, pitch, roll] in degrees; gyroscope readings are remapped through this transform in `update_orientation()`
- `MotionController._offset_left` / `_offset_right` instance attributes — motor encoder baselines captured at `run_motor_update()` startup so all reported positions are zero-referenced from that point

### Changed
- `Map.update_obstacles()` in `modules/navigation_system.py` — magnetic hazard now mapped at `state.sensors.mag_world_position` (magnetometer world position) instead of the robot's base position
- `Map.update_obstacles()` — IR heat sources now mapped independently at each element's own `world_position`; each element is checked against `ir_threshold` separately
- `Location._compute_sensor_poses()` — computes separate world positions for `ir_sensor_left` and `ir_sensor_right` using their respective `local_position` offsets; computes `mag_world_position` from `config.sensors.imu.local_position`
- `SensorInput.__init__()` — `IRSensor` now constructed with `ir_sensor_left.pin` and `ir_sensor_right.pin` instead of `ir_sensor_left.pin + 1` hardcoded
- `SensorInput.get_ir_values()` and `update_state()` write to `ir_sensor_left.value` / `ir_sensor_right.value` instead of the former `ir_sensor.value1` / `ir_sensor.value2`
- `modules/config.py` module docstring tree updated to reflect `ir_sensor_left`, `ir_sensor_right`, and `imu` fields on `SensorConfig`
- `MapConfig` in `modules/config.py` — `map_width`/`map_height` replaced by explicit `x_min`, `x_max`, `y_min`, `y_max` bounds (each defaulting to ±10.0 m); `resolution` default changed from 0.01 to 0.10 (10 cm/cell); `origin` field removed; `unit_length` default changed from 1 to 10
- `UltrasonicSensorConfig` default mount transforms in `SensorConfig` updated to physical hardware offsets: left sensor pin 22 at [0.04, 0.115, 0.05] / orientation [-90, 0, 0]; right sensor pin 26 at [0.04, -0.115, 0.05] / orientation [90, 0, 0]; center sensor pin 24 at [0.16, 0.0, 0.04] / orientation [0, 0, 0]
- `Location.__init__()` — `initial_heading` from `config` is written into the yaw component of `initial_orientation` rather than using `initial_orientation[0]` directly
- `Location.update_orientation()` — raw gyro vector is now rotated through `config.sensors.imu.local_orientation` before integration; `discrete_orientation` is updated at the end of every call
- `Navigation.get_obstacle_positions()` — obstacle positions now computed from the robot's `discrete_orientation` rather than `sensor.world_orientation`; each sensor's facing vector is derived from the discrete heading (center=forward, left=CCW 90°, right=CW 90°)
- `Navigation.go_to()` — signature changed to accept a grid cell `(col, row)` instead of a world-frame coordinate; marks the destination cell as value 4 (exit) on the map; navigation is driven by `automatic_turn()` in an async loop that cancels when the robot reaches the destination cell
- `Map.update_obstacles()` — obstacle detection now uses priority-based grid updates: higher-priority cell values (magnetic > heat > path) are only written if the existing cell value is lower priority
- `Map.save_map()` — wall cells (value 6) are exported as 0 (free) in the CSV so structural boundary cells are not written to the output file
- `Navigation.run_navigation_update()` — now updates `state.nav.obstacle_neighbors` by calling `Map.get_obstacle_states()` each tick
- `MotionController._sync_state()` — positions are reported relative to the encoder baseline captured at startup (`get_position() - _offset_left/right`)
- `MotionController.forward()` / `backward()` — motor direction signs corrected to match physical wiring (left: +s, right: -s for forward)
- `MotionController.turn_left()` / `turn_right()` — implementations swapped to match corrected wiring convention
- `Transformation._yaw_matrix()`, `_pitch_matrix()`, `_roll_matrix()` — rotation matrix entries corrected to standard right-hand-rule convention
- `Location._euler_from_matrix()` — matrix index references corrected for the fixed rotation matrix convention
- `__main__` entry point in `modules/navigation_system.py` — now prompts for a destination grid cell (col, row) and passes it to `go_to()`; map is always saved on exit; display loop simplified to show grid cell, discrete neighbors, and cell name labels; `main()` accepts `dest_cell` argument

### Removed
- `SensorConfig.ir_sensor` (`IRSensorConfig`) — replaced by `ir_sensor_left` and `ir_sensor_right`
- `IRSensorState.value1` / `IRSensorState.value2` — replaced by a single `value` field; left and right elements are now separate `IRSensorState` instances
- `MapConfig.origin` field — grid origin is now computed from `x_min`/`y_min` and `resolution` at `Map.__init__()` time
- `Navigation.go_to()` world-coordinate phase-1/phase-2 logic (rotate-then-drive) — replaced by `automatic_turn()`-based cell navigation

## [0.3.0] - 2026-04-25

### Added
- `modules/config.py` — new configuration module with `RobotConfig`, `MotorConfig`, `SensorConfig`, `UltrasonicSensorConfig`, and `IRSensorConfig` dataclasses; centralizes all static hardware configuration (pins, mount offsets, wheel diameter, motor ports, angle tolerance, initial pose)
- `basehat/IRSensor.py` — `IRSensor` class for dual-channel analog IR sensing via Grove BaseHAT ADC
- `basehat/HallSensor.py` — `HallSensor` class wrapping `gpiozero.Button` for Hall-effect magnet detection
- `basehat/LightSensor.py` — `LightSensor` class for Grove analog light sensing (0–1000 range)
- `basehat/template.py` — contributor template for new sensor/motor basehat modules
- `examples/IRSensor_Example.py` — standalone usage example for `IRSensor`
- `IRSensorState` dataclass in `modules/state.py` holding `value1`, `value2` (0–999 IR readings), and `world_position`
- `SensorState.ir_sensor` field (`IRSensorState`) in `modules/state.py`
- `SensorInput.ir_sensor` attribute, `get_ir_values()`, and `has_ir_sensor()` in `modules/sensors.py`; `update_state()` writes both IR values to `state.sensors.ir_sensor`
- `_flush_serial()` helper in `modules/motion.py` — discards accumulated BuildHAT serial buffer data before `Motor` initialization to prevent `BuildHATError` on startup
- IR sensor world-position computation in `Location._compute_sensor_poses()` using `IRSensorConfig.local_position`

### Changed
- `SensorInput.__init__()`, `Location.__init__()`, `Navigation.__init__()`, and `MotionController.__init__()` signatures changed from `**kwargs` to an explicit `config: RobotConfig` parameter; all hardware options (pins, ports, speeds, tolerances, initial pose) are now read from `config` instead of keyword arguments
- `MotionController.__init__()` now calls `_flush_serial()` before constructing `Motor` instances; motor ports sourced from `config.motors.port_left/right`
- `UltrasonicSensorState` in `modules/state.py` stripped of `local_position` and `local_orientation` fields (static mount config moved to `UltrasonicSensorConfig` in `config.py`)
- `State.mode` field removed; angle unit mode is now owned by `RobotConfig.mode`
- `Location._compute_sensor_poses()` reads ultrasonic mount transforms from `RobotConfig.sensors` instead of `UltrasonicSensorState`; `Navigation.transformer` now uses `config.mode`
- `__main__` entry point in `modules/navigation_system.py` updated to instantiate `RobotConfig` and pass it to all components; display loop now shows IR values and magnetic field
- `modules/state.py` module docstring updated to clarify the state/config separation

### Renamed
- `basehat/button.py` → `basehat/Button.py`
- `basehat/imu_sensor.py` → `basehat/IMUSensor.py`
- `basehat/line_finder.py` → `basehat/LineFinder.py`
- `basehat/ultrasonic_sensor.py` → `basehat/UltrasonicSensor.py`

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
