"""                                                                                                                                                
navigation_system.py

3D Navigation and Position Tracking System for MACRO.

This module provides:
- Transformation: 3D rotation and translation utilities using Euler angles
- Location: Position tracking using motor odometry and orientation data
- Navigation: Extended position tracking with timestamped logging

Classes:
    Transformation: Handles 3D coordinate transformations (rotation/translation)
    Location: Tracks position, velocity, and orientation using motor encoder data
    Navigation: Extends Location with continuous update loop and logging
"""

import asyncio
import numpy as np
import math
from .state import State
from .motion import MotionController


class Transformation:
    """
    3D transformation utilities for rotation and translation operations.
    
    Supports Euler angle rotations (yaw, pitch, roll) and vector operations.
    All rotation matrices follow the right-hand rule convention.
    
    Args:
        mode (str): Angle unit mode - "degrees" (default) or "radians"
    """
    
    def __init__(self, **kwargs):
        self.mode = kwargs.get("mode", "degrees")

    async def get_rotation_yaw(self, **kwargs):
        """Generate rotation matrix for yaw (Z-axis rotation)."""
        yaw = kwargs.get("yaw", 0.0)
        invert = kwargs.get("invert", False)
        
        if self.mode == "degrees":
            yaw = math.radians(yaw)

        R_yaw = np.array([
            [ math.cos(yaw),  math.sin(yaw), 0],
            [-math.sin(yaw),  math.cos(yaw), 0],
            [0,               0,             1]
        ])
        return np.transpose(R_yaw) if invert else R_yaw
    
    async def get_rotation_pitch(self, **kwargs):
        """Generate rotation matrix for pitch (Y-axis rotation)."""
        pitch = kwargs.get("pitch", 0.0)
        invert = kwargs.get("invert", False)
        
        if self.mode == "degrees":
            pitch = math.radians(pitch)

        R_pitch = np.array([
            [ math.cos(pitch), 0, -math.sin(pitch)],
            [0,                1,  0              ],
            [ math.sin(pitch), 0,  math.cos(pitch)]
        ])
        return np.transpose(R_pitch) if invert else R_pitch
    
    async def get_rotation_roll(self, **kwargs):
        """Generate rotation matrix for roll (X-axis rotation)."""
        roll = kwargs.get("roll", 0.0)
        invert = kwargs.get("invert", False)
        
        if self.mode == "degrees":
            roll = math.radians(roll)

        R_roll = np.array([
            [1, 0,               0              ],
            [0, math.cos(roll),  math.sin(roll) ],
            [0, -math.sin(roll), math.cos(roll) ]
        ])
        return np.transpose(R_roll) if invert else R_roll
    
    async def get_rotation(self, **kwargs):
        """
        Generate combined rotation matrix from yaw, pitch, and roll.
        
        Rotation order: Yaw -> Pitch -> Roll (ZYX convention)
        """
        orientation = kwargs.get("orientation", [0.0, 0.0, 0.0])
        invert = kwargs.get("invert", False)
        
        R_yaw = await self.get_rotation_yaw(yaw=orientation[0], invert=invert)
        R_pitch = await self.get_rotation_pitch(pitch=orientation[1], invert=invert)
        R_roll = await self.get_rotation_roll(roll=orientation[2], invert=invert)
        
        return np.matmul(R_yaw, np.matmul(R_pitch, R_roll))
    
    async def rotate_vector(self, **kwargs):
        """Apply rotation to a 3D vector."""
        vector = np.array(kwargs.get("vector", [0.0, 0.0, 0.0]))
        orientation = kwargs.get("orientation", [0.0, 0.0, 0.0])
        invert = kwargs.get("invert", False)
        
        R = await self.get_rotation(orientation=orientation, invert=invert)
        return np.matmul(R, vector)
    
    async def translate_vector(self, **kwargs):
        """Apply translation to a 3D vector."""
        vector = np.array(kwargs.get("vector", [0.0, 0.0, 0.0]))
        translation = np.array(kwargs.get("translation", [0.0, 0.0, 0.0]))
        return vector + translation


class Location:
    """
    3D position tracking using motor odometry and gyroscope data.
    
    Uses motor encoder displacement and wheel diameter to estimate position.
    Orientation is updated from the IMU gyroscope, but accelerometer data is
    not used for position calculations.
    Reads raw sensor data from State, which is updated by SensorInput.
    
    Args:
        state (State): Centralized state object for sensor and navigation data
        position (list): Initial position [x, y, z] in meters
        orientation (list): Initial orientation [yaw, pitch, roll] in degrees
        wheel_diameter (float): Wheel diameter in meters (default: 0.056)
        mode (str): Angle unit mode - "degrees" (default) or "radians"
    
    Attributes:
        state.nav.position: Current position [x, y, z] in global frame
        state.nav.velocity: Current velocity [vx, vy, vz] in global frame
        state.nav.orientation: Current orientation [yaw, pitch, roll]
    """
    
    def __init__(self, state: State, **kwargs):
        self.state = state
        self.wheel_diameter = kwargs.get("wheel_diameter", 0.056)
        self.mode = kwargs.get("mode", "degrees")

        # Seed nav state from kwargs
        self.state.nav.position = np.array(kwargs.get("position", [0.0, 0.0, 0.0]))
        self.state.nav.velocity = np.zeros(3)
        self.state.nav.acceleration = np.zeros(3)
        self.state.nav.orientation = np.array(kwargs.get("orientation", [0.0, 0.0, 0.0]))
        self.state.nav.angular_velocity = np.zeros(3)
        self.state.nav.angular_acceleration = np.zeros(3)

        # Track previous motor positions to compute deltas
        self._prev_motor_left = state.motor_left.position
        self._prev_motor_right = state.motor_right.position

        self.transformer = Transformation(mode=self.mode)
    
    async def update_orientation(self, dt: float = 0.1):
        gyro = self.state.sensors.angular_velocity_raw

        self.state.nav.orientation += (
            0.5 * self.state.nav.angular_acceleration * dt ** 2
            + self.state.nav.angular_velocity * dt
        )
        self.state.nav.angular_acceleration = (gyro - self.state.nav.angular_velocity) / dt
        self.state.nav.angular_velocity = gyro

        return True
    
    async def update_position(self, dt: float = 0.1):
        delta_left = self.state.motor_left.position - self._prev_motor_left
        delta_right = self.state.motor_right.position - self._prev_motor_right
        delta_degrees = (delta_left + delta_right) / 2
        distance = (delta_degrees / 360) * math.pi * self.wheel_diameter

        self._prev_motor_left = self.state.motor_left.position
        self._prev_motor_right = self.state.motor_right.position

        speed = distance / dt if dt > 0 else 0.0
        velocity = await self.transformer.rotate_vector(
            vector=[speed, 0.0, 0.0],
            orientation=self.state.nav.orientation
        )

        self.state.nav.position += (
            0.5 * self.state.nav.acceleration * dt ** 2
            + self.state.nav.velocity * dt
        )
        self.state.nav.acceleration = (velocity - self.state.nav.velocity) / dt
        self.state.nav.velocity = velocity

        return True
    
    async def update(self, dt: float = 0.1):
        """Update both orientation and position."""
        await self.update_orientation(dt=dt)
        await self.update_position(dt=dt)
        return True
    
class Navigation:
    def __init__(self, state: State, motion: MotionController, **kwargs):
        self.state = state
        self.motion = motion
        self.angle_tolerance = kwargs.get("angle_tolerance", 5.0)
        self._log = []

    async def go_to(self, destination):
        """
        Navigate to a 2D destination by first rotating to face it, then driving forward.

        Args:
            destination: Target position [x, y] or [x, y, z]. Only x/y are used.
        """
        dest_x = float(destination[0])
        dest_y = float(destination[1])

        # Phase 1: Rotate to face the destination.
        while True:
            cur_x = self.state.nav.position[0]
            cur_y = self.state.nav.position[1]
            dx = dest_x - cur_x
            dy = dest_y - cur_y

            target_yaw = math.degrees(math.atan2(dy, dx))
            current_yaw = self.state.nav.orientation[0]
            angle_error = (target_yaw - current_yaw + 180) % 360 - 180

            if abs(angle_error) <= self.angle_tolerance:
                self.motion.stop()
                break

            if angle_error > 0:
                self.motion.turn_left()
            else:
                self.motion.turn_right()

            await asyncio.sleep(0)

        # Snapshot position after the turn to measure travel distance.
        start = np.array([self.state.nav.position[0], self.state.nav.position[1]])
        total_distance = math.sqrt((dest_x - start[0]) ** 2 + (dest_y - start[1]) ** 2)

        # Phase 2: Drive forward until the required distance is covered.
        while True:
            cur = np.array([self.state.nav.position[0], self.state.nav.position[1]])
            traveled = float(np.linalg.norm(cur - start))

            if traveled >= total_distance:
                self.motion.stop()
                break

            self.motion.forward()
            await asyncio.sleep(0)