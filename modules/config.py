"""
config.py

Static hardware and system configuration for MACRO.

All values here are set once at startup and never mutated at runtime.
Dynamic runtime values (sensor readings, computed pose, motor state) live
in state.py.

Structure:
    RobotConfig
    ├── mode: str                       — angle unit ("degrees" or "radians")
    ├── wheel_diameter: float           — meters
    ├── motor_speed: int                — default drive speed
    ├── angle_tolerance: float          — heading error threshold for navigation
    ├── initial_position: np.ndarray    — starting [x, y, z] in meters
    ├── initial_orientation: np.ndarray — starting [yaw, pitch, roll]
    ├── motors: MotorConfig
    │   ├── port_left: str              — BuildHAT port for left motor ("A"–"D")
    │   └── port_right: str             — BuildHAT port for right motor ("A"–"D")
    ├── sensors: SensorConfig
    │   └── ...
    └── map: MapConfig
        ├── x_min: float                — mapped area minimum x in meters
        ├── x_max: float                — mapped area maximum x in meters
        ├── y_min: float                — mapped area minimum y in meters
        ├── y_max: float                — mapped area maximum y in meters
        ├── resolution: float           — meters per grid cell
        ├── magnetic_threshold: float   — field strength to mark magnetic source
        └── ir_threshold: int           — IR value (0–999) to mark heat source
        ├── enable_imu: bool
        ├── enable_ultrasonic: bool
        ├── enable_button: bool
        ├── enable_ir_sensor: bool
        ├── button_pin: int
        ├── ultrasonic_left: UltrasonicSensorConfig
        ├── ultrasonic_right: UltrasonicSensorConfig
        ├── ultrasonic_center: UltrasonicSensorConfig
        ├── ir_sensor_left: IRSensorConfig
        ├── ir_sensor_right: IRSensorConfig
        └── imu: IMUSensorConfig
"""

from __future__ import annotations
from dataclasses import dataclass, field
from typing import Optional, Tuple
import numpy as np


def _zero_vector() -> np.ndarray:
    return np.zeros(3, dtype=float)


@dataclass
class MotorConfig:
    """Static configuration for the drive motors.

    Attributes:
        port_left: BuildHAT port letter for the left motor (default: "A").
        port_right: BuildHAT port letter for the right motor (default: "B").
    """
    port_left: str = "A"
    port_right: str = "B"


@dataclass
class UltrasonicSensorConfig:
    """Static configuration for a single ultrasonic sensor.

    Attributes:
        pin: GPIO pin number.
        local_position: Sensor origin offset from IMU in robot frame, meters [x, y, z].
        local_orientation: Sensor axis orientation relative to IMU frame, [yaw, pitch, roll] degrees.
    """
    pin: int = 0
    local_position: np.ndarray = field(default_factory=_zero_vector)
    local_orientation: np.ndarray = field(default_factory=_zero_vector)


@dataclass
class IRSensorConfig:
    """Static configuration for a single IR sensor element.

    Attributes:
        pin: Analog port number for this element.
        local_position: Sensor element origin offset from robot base in robot frame, meters [x, y, z].
    """
    pin: int = 0
    local_position: np.ndarray = field(default_factory=_zero_vector)


@dataclass
class IMUSensorConfig:
    """Static configuration for the IMU mount relative to the robot frame.

    Attributes:
        local_position: IMU origin offset from robot base in robot frame,
            meters [x, y, z].
        local_orientation: Rotation from IMU frame to robot frame as
            [yaw, pitch, roll] in degrees (ZYX Euler, extrinsic).
            Gyroscope readings are remapped by this transform so the output
            is expressed in the robot body frame.
    """
    local_position: np.ndarray = field(default_factory=_zero_vector)
    local_orientation: np.ndarray = field(default_factory=_zero_vector)


@dataclass
class SensorConfig:
    """Static configuration for all sensors.

    Attributes:
        enable_imu: Enable IMU sensor (default: True).
        enable_ultrasonic: Enable all three ultrasonic sensors (default: True).
        enable_button: Enable button (default: False).
        enable_ir_sensor: Enable IR sensor (default: False).
        button_pin: GPIO pin for button (default: 22).
        ultrasonic_left: Config for left ultrasonic sensor (default pin: 16).
        ultrasonic_right: Config for right ultrasonic sensor (default pin: 5).
        ultrasonic_center: Config for center ultrasonic sensor (default pin: 26).
        ir_sensor_left: Config for the left IR element (pin, local_position).
        ir_sensor_right: Config for the right IR element (local_position; uses pin+1 automatically).
    """
    enable_imu: bool = True
    enable_ultrasonic: bool = True
    enable_button: bool = False
    enable_ir_sensor: bool = True
    button_pin: int = 22

    # Mount transforms are in the robot frame with the IMU at the origin.
    # local_position: [x, y, z] offset in meters.
    # local_orientation: [yaw, pitch, roll] in degrees.
    # Update these to match the physical sensor placement on the robot.
    imu: IMUSensorConfig = field(
        default_factory=lambda: IMUSensorConfig(
            local_position=np.array([0.0, 0.0, 0.0]),
        )
    )
    ultrasonic_left: UltrasonicSensorConfig = field(
        default_factory=lambda: UltrasonicSensorConfig(
            pin=22,
            local_position=np.array([0.04, 0.115, 0.05]),
            local_orientation=np.array([-90.0, 0.0, 0.0]),
        )
    )
    ultrasonic_right: UltrasonicSensorConfig = field(
        default_factory=lambda: UltrasonicSensorConfig(
            pin=26,
            local_position=np.array([0.04, -0.115, 0.05]),
            local_orientation=np.array([90.0, 0.0, 0.0]),
        )
    )
    ultrasonic_center: UltrasonicSensorConfig = field(
        default_factory=lambda: UltrasonicSensorConfig(
            pin=24,
            local_position=np.array([0.16, 0.0, 0.04]),
            local_orientation=np.array([0.0, 0.0, 0.0]),
        )
    )
    ir_sensor_left: IRSensorConfig = field(
        default_factory=lambda: IRSensorConfig(
            pin=2,
            local_position=np.array([0.0, 0.0, 0.0]),
        )
    )
    ir_sensor_right: IRSensorConfig = field(
        default_factory=lambda: IRSensorConfig(
            pin=3,
            local_position=np.array([0.0, 0.0, 0.0]),
        )
    )


@dataclass
class MapConfig:
    """Static configuration for the 2D occupancy grid map.

    Attributes:
        x_min: Minimum x coordinate of the mapped area in meters (default: -10.0).
        x_max: Maximum x coordinate of the mapped area in meters (default: 10.0).
        y_min: Minimum y coordinate of the mapped area in meters (default: -10.0).
        y_max: Maximum y coordinate of the mapped area in meters (default: 10.0).
        resolution: Meters per grid cell (default: 0.01 → 1 cm/cell).
        magnetic_threshold: Magnetic field reading above which a cell is marked
            as a magnetic source (default: 1000.0).
        ir_threshold: IR sensor value (0–999) above which the sensor position is
            marked as a heat source (default: 500).
        team: Team identifier written to the map CSV header (default: 0).
        unit_length: Physical size of one grid cell written to the CSV header
            (default: 1).
        unit: Unit string for unit_length written to the CSV header (default: "cm").
    """
    x_min: float = -10.0
    x_max: float = 10.0
    y_min: float = -10.0
    y_max: float = 10.0
    resolution: float = 0.01
    magnetic_threshold: float = 1000.0
    ir_threshold: int = 500
    team: int = 0
    unit_length: int = 1
    unit: str = "cm"


@dataclass
class RobotConfig:
    """Top-level static configuration for GEARS robot.

    Attributes:
        mode: Angle unit mode — "degrees" (default) or "radians".
        wheel_diameter: Wheel diameter in meters (default: 0.056).
        motor_speed: Default motor drive speed (default: 50).
        angle_tolerance: Heading error in degrees considered aligned for navigation (default: 5.0).
        initial_position: Starting position [x, y, z] in meters.
        initial_heading: Starting heading of the robot in degrees, measured CCW from +x.
            Sets the yaw component of the initial orientation (default: 0.0).
        initial_orientation: Starting orientation [yaw, pitch, roll].
        motors: Motor port configuration.
        sensors: Sensor hardware and mount configuration.
        map: Occupancy grid map configuration.
    """
    mode: str = "degrees"
    wheel_diameter: float = 0.056
    motor_speed: int = 50
    angle_tolerance: float = 5.0
    initial_position: np.ndarray = field(default_factory=_zero_vector)
    initial_heading: float = 0.0
    initial_orientation: np.ndarray = field(default_factory=_zero_vector)
    motors: MotorConfig = field(default_factory=MotorConfig)
    sensors: SensorConfig = field(default_factory=SensorConfig)
    map: MapConfig = field(default_factory=MapConfig)
