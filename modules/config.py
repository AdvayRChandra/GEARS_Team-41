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
    └── sensors: SensorConfig
        ├── enable_imu: bool
        ├── enable_ultrasonic: bool
        ├── enable_button: bool
        ├── enable_ir_sensor: bool
        ├── button_pin: int
        ├── ultrasonic_left: UltrasonicSensorConfig
        ├── ultrasonic_right: UltrasonicSensorConfig
        ├── ultrasonic_center: UltrasonicSensorConfig
        └── ir_sensor: IRSensorConfig
"""

from __future__ import annotations
from dataclasses import dataclass, field
import numpy as np


def _zero_vector() -> np.ndarray:
    return np.zeros(3, dtype=float)


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
    """Static configuration for the IR sensor.

    Attributes:
        pin: Analog port number; pin+1 is used automatically for value2.
        local_position: Sensor origin offset from IMU in robot frame, meters [x, y, z].
    """
    pin: int = 0
    local_position: np.ndarray = field(default_factory=_zero_vector)


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
        ir_sensor: Config for IR sensor (default pin: 0).
    """
    enable_imu: bool = True
    enable_ultrasonic: bool = True
    enable_button: bool = False
    enable_ir_sensor: bool = False
    button_pin: int = 22

    # Mount transforms are in the robot frame with the IMU at the origin.
    # local_position: [x, y, z] offset in meters.
    # local_orientation: [yaw, pitch, roll] in degrees.
    # Update these to match the physical sensor placement on the robot.
    ultrasonic_left: UltrasonicSensorConfig = field(
        default_factory=lambda: UltrasonicSensorConfig(
            pin=16,
            local_position=np.array([0.0, 0.0, 0.0]),
            local_orientation=np.array([90.0, 0.0, 0.0]),
        )
    )
    ultrasonic_right: UltrasonicSensorConfig = field(
        default_factory=lambda: UltrasonicSensorConfig(
            pin=5,
            local_position=np.array([0.0, 0.0, 0.0]),
            local_orientation=np.array([-90.0, 0.0, 0.0]),
        )
    )
    ultrasonic_center: UltrasonicSensorConfig = field(
        default_factory=lambda: UltrasonicSensorConfig(
            pin=26,
            local_position=np.array([0.0, 0.0, 0.0]),
            local_orientation=np.array([0.0, 0.0, 0.0]),
        )
    )
    ir_sensor: IRSensorConfig = field(
        default_factory=lambda: IRSensorConfig(
            pin=0,
            local_position=np.array([0.0, 0.0, 0.0]),
        )
    )


@dataclass
class RobotConfig:
    """Top-level static configuration for MACRO.

    Attributes:
        mode: Angle unit mode — "degrees" (default) or "radians".
        wheel_diameter: Wheel diameter in meters (default: 0.056).
        motor_speed: Default motor drive speed (default: 50).
        angle_tolerance: Heading error in degrees considered aligned for navigation (default: 5.0).
        initial_position: Starting position [x, y, z] in meters.
        initial_orientation: Starting orientation [yaw, pitch, roll].
        sensors: Sensor hardware and mount configuration.
    """
    mode: str = "degrees"
    wheel_diameter: float = 0.056
    motor_speed: int = 50
    angle_tolerance: float = 5.0
    initial_position: np.ndarray = field(default_factory=_zero_vector)
    initial_orientation: np.ndarray = field(default_factory=_zero_vector)
    sensors: SensorConfig = field(default_factory=SensorConfig)
