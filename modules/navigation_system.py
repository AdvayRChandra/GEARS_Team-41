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
from modules.state import State


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
        yaw = kwargs.get("yaw", 0.0)
        pitch = kwargs.get("pitch", 0.0)
        roll = kwargs.get("roll", 0.0)
        invert = kwargs.get("invert", False)
        
        R_yaw = await self.get_rotation_yaw(yaw=yaw, invert=invert)
        R_pitch = await self.get_rotation_pitch(pitch=pitch, invert=invert)
        R_roll = await self.get_rotation_roll(roll=roll, invert=invert)
        
        return np.matmul(R_yaw, np.matmul(R_pitch, R_roll))
    
    async def rotate_vector(self, **kwargs):
        """Apply rotation to a 3D vector."""
        vector = np.array(kwargs.get("vector", [0.0, 0.0, 0.0]))
        yaw = kwargs.get("yaw", 0.0)
        pitch = kwargs.get("pitch", 0.0)
        roll = kwargs.get("roll", 0.0)
        invert = kwargs.get("invert", False)
        
        R = await self.get_rotation(yaw=yaw, pitch=pitch, roll=roll, invert=invert)
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
        state (State): Centralized state object for sensor data
        position (list): Initial position [x, y, z] in meters
        orientation (list): Initial orientation [yaw, pitch, roll] in degrees
        mode (str): Angle unit mode - "degrees" (default) or "radians"
    
    Attributes:
        state.position: Current position [x, y, z] in global frame
        state.velocity: Current velocity [vx, vy, vz] in global frame
        state.orientation: Current orientation [yaw, pitch, roll]
    """
    
    def __init__(self, **kwargs):
        # Initialize State dataclass (accept external state or create new)
        self.state = kwargs.get("state", State(
            position=np.array(kwargs.get("position", [0.0, 0.0, 0.0])),
            orientation=np.array(kwargs.get("orientation", [0.0, 0.0, 0.0])),
            sensor_positions={
                "imu": np.array([0.0, 0.0, 0.0]),
                "lf_left": np.array([0.0, 0.0, 0.0]),
                "lf_right": np.array([0.0, 0.0, 0.0]),
                "color_sensor": np.array([0.0, 0.0, 0.0]),
                "cargo_deploy": np.array([0.0, 0.0, 0.0])
            }
        ))
        
        # Initialize sensor_positions if not set
        if self.state.sensor_positions is None:
            self.state.sensor_positions = {
                "imu": np.array([0.0, 0.0, 0.0]),
                "lf_left": np.array([0.0, 0.0, 0.0]),
                "lf_right": np.array([0.0, 0.0, 0.0]),
                "color_sensor": np.array([0.0, 0.0, 0.0]),
                "cargo_deploy": np.array([0.0, 0.0, 0.0])
            }

        # Previous state values for calculations
        self.prev_position = self.state.position.copy()
        self.prev_orientation = self.state.orientation.copy()

        # Sensor position offsets
        self.imu_to_ground = np.array([0.0, 0.0, -kwargs.get("imu_height", 0.015)])
        self.ground_to_lf_left = np.array([kwargs.get("imu_to_lf", 0.125), kwargs.get("lf_offset", 0.0225), kwargs.get("lf_height", 0.025)])
        self.ground_to_lf_right = np.array([kwargs.get("imu_to_lf", 0.125), -kwargs.get("lf_offset", 0.0225), kwargs.get("lf_height", 0.025)])
        self.ground_to_color = np.array([-kwargs.get("imu_to_color", 0.11), 0.0, kwargs.get("color_sensor_height", 0.025)])
        self.ground_to_cargo = np.array([-kwargs.get("imu_to_cargo", 0.24), 0.0, 0.0])

        # Calibration offsets (measured when stationary)
        if self.state.bias is None:
            self.state.bias = {
                "accel": np.array([0.0, 0.0, 0.0]),
                "gyro": np.array([0.0, 0.0, 0.0]),
                "mag": 0.0
            }

        # Velocity decay factor to reduce drift (0.0 = no decay, 1.0 = instant stop)
        self.velocity_decay = kwargs.get("velocity_decay", 0.4)
        
        # Motor velocity threshold for velocity decay (degrees/second)
        self.motor_velocity_threshold = kwargs.get("motor_velocity_threshold", 1.0)

        # Wheel diameter used for motor odometry
        self.wheel_diameter = kwargs.get("wheel_diameter", self.state.wheel_diameter)
        self.wheel_radius = self.wheel_diameter / 2.0
        self.prev_motor_position = self.state.motor_position
        
        # Components
        self.transformer = Transformation(**kwargs)
    
    async def update_orientation(self, **kwargs):
        """
        Update orientation by integrating gyroscope data from State.
        
        Args:
            dt (float): Time step in seconds
        """
        dt = kwargs.get("dt", 0.1)
        
        # Get angular velocity from State (updated by SensorInput)
        # State stores as [gx, gy, gz], convert to [yaw, pitch, roll]
        raw_gyro = self.state.angular_velocity_raw
        angular_velocity = np.array([raw_gyro[2], raw_gyro[1], raw_gyro[0]])  # yaw, pitch, roll

        # Subtract gyro bias if calibrated (reorder bias from [gx, gy, gz] to [gz, gy, gx])
        if self.state.calibrated_orientation:
            gyro_bias = self.state.bias["gyro"]
            angular_velocity -= np.array([gyro_bias[2], gyro_bias[1], gyro_bias[0]])
        
        self.state.angular_velocity = angular_velocity
        self.state.orientation += angular_velocity * dt

        return True
    
    async def update_imu_position(self, **kwargs):
        """
        Update position using motor odometry and wheel diameter.

        Uses the change in motor position and the configured wheel diameter
        to compute linear displacement. Orientation remains driven by the
        IMU gyroscope, but accelerometer values are not used for position.

        Args:
            dt (float): Time step in seconds
            display (bool): Print position data if True
        """
        dt = kwargs.get("dt", 0.1)
        display = kwargs.get("display", False)

        # Compute linear displacement from motor rotation
        current_motor_position = self.state.motor_position
        delta_motor_degrees = current_motor_position - self.prev_motor_position
        self.prev_motor_position = current_motor_position

        distance = math.radians(delta_motor_degrees) * self.wheel_radius
        self.state.distance_traveled += abs(distance) * 100.0

        translation_local = np.array([distance, 0.0, 0.0])
        translation_global = await self.transformer.rotate_vector(
            vector=translation_local,
            yaw=self.state.orientation[0],
            pitch=self.state.orientation[1],
            roll=self.state.orientation[2],
            invert=False
        )

        self.state.position = await self.transformer.translate_vector(
            vector=self.state.position,
            translation=translation_global
        )

        # Estimate velocity from motor angular velocity and wheel radius
        linear_velocity = math.radians(self.state.motor_velocity) * self.wheel_radius
        self.state.velocity = await self.transformer.rotate_vector(
            vector=np.array([linear_velocity, 0.0, 0.0]),
            yaw=self.state.orientation[0],
            pitch=self.state.orientation[1],
            roll=self.state.orientation[2],
            invert=False
        )

        # Apply velocity decay when motors are effectively stationary
        if abs(self.state.motor_velocity) < self.motor_velocity_threshold:
            self.state.velocity *= (1.0 - self.velocity_decay)

        if display:
            print(f"Position: {self.state.position}, Velocity: {self.state.velocity}")

        return True
    
    async def update_positions_from_imu(self, **kwargs):
        """
        Update all sensor positions relative to the ground position.
        Ground position is already calculated in update_imu_position.
        """
        sensor_offsets = {
            "lf_left": self.ground_to_lf_left,
            "lf_right": self.ground_to_lf_right,
            "color_sensor": self.ground_to_color,
            "cargo_deploy": self.ground_to_cargo
        }

        for sensor, offset in sensor_offsets.items():
            self.state.sensor_positions[sensor] = await self.transformer.translate_vector(
                vector=self.state.position,
                translation=await self.transformer.rotate_vector(
                    vector=offset,
                    yaw=self.state.orientation[0],
                    pitch=self.state.orientation[1],
                    roll=self.state.orientation[2],
                    invert=False
                )
            )

    async def update_position(self, **kwargs):
        """
        Update position based on motor odometry and current orientation.

        Args:
            dt (float): Time step in seconds
            display (bool): Print position data if True
        """
        dt = kwargs.get("dt", 0.1)
        display = kwargs.get("display", False)

        await self.update_imu_position(dt=dt, display=display)
        await self.update_positions_from_imu()

        return True

    def get_position(self):
        """
        Get the current position as a tuple.
        
        Returns:
            tuple: Current position (x, y, z) in meters
        """
        return tuple(self.state.position)

    async def update(self, dt):
        """
        Update the location state based on motor odometry and gyroscope data.
        Assumes SensorInput is running and updating State.

        Args:
            dt (float): Time step in seconds.
        """
        await self.update_position(dt=dt)
        await self.update_orientation(dt=dt)


class Navigation(Location):
    """
    3D navigation system.
    
    Extends Location with state updates and display functionality.
    Magnetic field handling is done by the Cargo class.
    
    Args:
        Inherits all arguments from Location
    """
    
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
    
    async def update_state(self, **kwargs):
        """
        Update navigation state: position and orientation.
        
        Args:
            dt (float): Time step in seconds (default: 0.1)
        """
        dt = kwargs.get("dt", 0.1)

        # Update previous state values
        self.prev_position = self.state.position.copy()
        self.prev_orientation = self.state.orientation.copy()

        # Update current state (position and orientation only)
        # These methods update state in-place
        await self.update_position(dt=dt)
        await self.update_orientation(dt=dt)
    
    async def run_continuous_update(self, **kwargs):
        """
        Continuously update navigation state at a fixed interval.
        Calibration should be done via SensorInput before calling this.
        
        Args:
            update_interval (float): Update interval in seconds (default: 0.1)
        """
        update_interval = kwargs.get("update_interval", 0.1)
        
        while True:
            await self.update_state(dt=update_interval)
            await asyncio.sleep(update_interval)


if __name__ == "__main__":
    # Example usage
    from modules.sensors import SensorInput
    from modules.state import State
    
    # Create shared state
    state = State()
    
    # Create sensors with shared state
    sensors = SensorInput(imu=True, state=state)
    
    # Create navigator with shared state
    navigator = Navigation(state=state, mode="degrees")
    
    async def main():
        # Start sensor update loop
        sensor_task = asyncio.create_task(sensors.run_sensor_update())
        
        # Allow sensors to start updating
        await asyncio.sleep(0.1)
        
        # Calibrate IMU via sensors module
        await sensors.calibrate_imu()
        
        try:
            await navigator.run_continuous_update(update_interval=0.1)
        finally:
            sensor_task.cancel()
    
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Stopping navigation.")