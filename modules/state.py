"""
State container for centralized sensor and navigation data.

This module defines dataclasses used by SensorInput, MotionController,
and navigation subsystems to share current pose, motion, and raw sensor
values in a single coherent object.

Structure:
    State
    ├── sensors: SensorState        — raw readings from all hardware sensors
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
class SensorState:
    # IMU (raw)
    acceleration_raw: np.ndarray = field(default_factory=_zero_vector)
    angular_velocity_raw: np.ndarray = field(default_factory=_zero_vector)
    magnetic_field: float = 0.0

    # Peripheral sensors
    ultrasonic_distance: float = -1.0
    lf_left_value: float = 0.0
    lf_right_value: float = 0.0
    button_pressed: bool = False
    color_sensor_value: int = -1


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
    mode: str = "degrees"
