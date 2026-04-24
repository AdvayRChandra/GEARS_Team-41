"""
State container for centralized sensor and navigation data.

This module holds only values that change at runtime (sensor readings,
computed pose, motor state).  Static configuration (pins, mount offsets,
wheel diameter, etc.) lives in config.py.

Structure:
    State
    ├── sensors: SensorState        — raw and computed readings for all hardware sensors
    │   ├── ultrasonic_left: UltrasonicSensorState
    │   ├── ultrasonic_right: UltrasonicSensorState
    │   ├── ultrasonic_center: UltrasonicSensorState
    │   └── ir_sensor: IRSensorState
    ├── nav: NavigationState        — computed pose, velocity, and orientation
    ├── motor_left: MotorState      — left motor (port A) position and motion status
    └── motor_right: MotorState     — right motor (port B) position and motion status
"""

from __future__ import annotations
from dataclasses import dataclass, field
import numpy as np


def _zero_vector() -> np.ndarray:
    return np.zeros(3, dtype=float)


@dataclass
class MotorState:
    is_moving: bool = False
    position: float = 0.0


@dataclass
class UltrasonicSensorState:
    """Runtime state for a single ultrasonic sensor.

    Attributes:
        distance: Latest range reading in centimeters, or -1.0 if unavailable.
        world_position: Sensor origin in global frame, meters [x, y, z]. Updated each tick.
        world_orientation: Sensor heading in global frame, [yaw, pitch, roll] degrees. Updated each tick.
    """
    distance: float = -1.0
    world_position: np.ndarray = field(default_factory=_zero_vector)
    world_orientation: np.ndarray = field(default_factory=_zero_vector)


@dataclass
class IRSensorState:
    """Runtime state for the IR sensor.

    Attributes:
        value1: Latest reading from the left IR element (0-999), or -1 if unavailable.
        value2: Latest reading from the right IR element (0-999), or -1 if unavailable.
        world_position: Sensor origin in global frame, meters [x, y, z]. Updated each tick.
    """
    value1: int = -1  # 0-999
    value2: int = -1  # 0-999
    world_position: np.ndarray = field(default_factory=_zero_vector)


@dataclass
class SensorState:
    # IMU (gyroscope and magnetometer)
    angular_velocity_raw: np.ndarray = field(default_factory=_zero_vector)
    magnetic_field: float = 0.0

    # Ultrasonic sensors
    ultrasonic_left: UltrasonicSensorState = field(default_factory=UltrasonicSensorState)
    ultrasonic_right: UltrasonicSensorState = field(default_factory=UltrasonicSensorState)
    ultrasonic_center: UltrasonicSensorState = field(default_factory=UltrasonicSensorState)

    # IR sensor
    ir_sensor: IRSensorState = field(default_factory=IRSensorState)

    # Peripheral sensors
    button_pressed: bool = False


@dataclass
class NavigationState:
    position: np.ndarray = field(default_factory=_zero_vector)
    velocity: np.ndarray = field(default_factory=_zero_vector)
    acceleration: np.ndarray = field(default_factory=_zero_vector)
    orientation: np.ndarray = field(default_factory=_zero_vector)
    angular_velocity: np.ndarray = field(default_factory=_zero_vector)
    angular_acceleration: np.ndarray = field(default_factory=_zero_vector)


@dataclass
class State:
    sensors: SensorState = field(default_factory=SensorState)
    nav: NavigationState = field(default_factory=NavigationState)
    motor_left: MotorState = field(default_factory=MotorState)
    motor_right: MotorState = field(default_factory=MotorState)
