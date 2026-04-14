"""                                                                                                                                                
navigation_system.py

3D Navigation and Position Tracking System for MACRO.

This module provides:
- Transformation: 3D rotation and translation utilities using Euler angles
- Location: Position tracking using IMU sensor data with dead reckoning
- Navigation: Extended position tracking with timestamped logging

Classes:
    Transformation: Handles 3D coordinate transformations (rotation/translation)
    Location: Tracks position, velocity, and orientation using IMU integration
    Navigation: Extends Location with continuous update loop and logging
"""

import asyncio
import numpy as np
import math
from systems.state import State

# Gravity constant (m/s^2)
GRAVITY = 9.80235


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
    3D position tracking using IMU sensor data.
    
    Uses dead reckoning to estimate position by integrating acceleration
    and gyroscope data. Position is tracked in a global reference frame.
    Reads raw sensor data from State, which is updated by SensorInput.
    
    Args:
        state (State): Centralized state object for sensor data
        position (list): Initial position [x, y, z] in meters
        orientation (list): Initial orientation [yaw, pitch, roll] in degrees
        mode (str): Angle unit mode - "degrees" (default) or "radians"
    
    Attributes:
        state.position: Current position [x, y, z] in global frame
        state.velocity: Current velocity [vx, vy, vz] in global frame
        state.acceleration: Current acceleration [ax, ay, az] in global frame (gravity-compensated)
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
        
        # Threshold for considering acceleration as noise (m/s^2)
        self.accel_threshold = kwargs.get("accel_threshold", 0.05)
        
        # Motor velocity threshold for velocity decay (degrees/second)
        self.motor_velocity_threshold = kwargs.get("motor_velocity_threshold", 1.0)
        
        # Components
        self.transformer = Transformation(**kwargs)
        self.motion_controller = kwargs.get("motion_controller", None)
        self.initialized = False
    
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
        
        if not self.initialized:
            # First iteration: initialize rates without calculating acceleration
            self.state.angular_velocity = angular_velocity
            self.initialized = True
        else:
            # Integrate orientation using previous angular velocity
            self.state.orientation += 0.5 * self.state.angular_acceleration * (dt ** 2) + self.state.angular_velocity * dt
            self.state.angular_acceleration = (angular_velocity - self.state.angular_velocity) / dt
            self.state.angular_velocity = angular_velocity
        
        return True
    
    async def update_imu_position(self, **kwargs):
        """
        Update IMU position by integrating accelerometer data.
        
        Transforms local acceleration to global frame and subtracts gravity.
        Applies calibration bias and noise thresholding.
        The ground position is then calculated from the IMU position.
        
        Uses proper integration order:
        1. Update IMU position using previous velocity and acceleration
        2. Update velocity using previous acceleration
        3. Calculate new acceleration for next iteration
        4. Calculate ground position from IMU position
        
        Args:
            dt (float): Time step in seconds
            display (bool): Print position data if True
        """
        dt = kwargs.get("dt", 0.1)
        display = kwargs.get("display", False)
        
        # Store previous acceleration and velocity for integration
        prev_acceleration = self.state.acceleration.copy()
        prev_velocity = self.state.velocity.copy()
        
        # Update IMU position FIRST using previous velocity and acceleration
        # p = p0 + v0*dt + 0.5*a0*dt^2
        self.state.sensor_positions["imu"] = await self.transformer.translate_vector(
            vector=self.state.sensor_positions["imu"],
            translation=prev_velocity * dt + 0.5 * prev_acceleration * (dt ** 2)
        )
        
        # Calculate ground position from IMU position
        self.state.position = await self.transformer.translate_vector(
            vector=self.state.sensor_positions["imu"],
            translation=await self.transformer.rotate_vector(
                vector=self.imu_to_ground,
                yaw=self.state.orientation[0],
                pitch=self.state.orientation[1],
                roll=self.state.orientation[2],
                invert=False
            )
        )
        
        # Update velocity SECOND using previous acceleration
        # v = v0 + a0*dt
        self.state.velocity = await self.transformer.translate_vector(
            vector=prev_velocity,
            translation=prev_acceleration * dt
        )
        
        # Apply velocity decay to reduce drift when motor is near stationary
        if self.motion_controller is not None:
            if abs(self.state.motor_velocity) < self.motor_velocity_threshold:
                self.state.velocity *= (1.0 - self.velocity_decay)
        else:
            # Fallback to acceleration threshold if no motion controller
            if np.linalg.norm(prev_acceleration) < self.accel_threshold:
                self.state.velocity *= (1.0 - self.velocity_decay)
        
        # Read new acceleration from State (updated by SensorInput)
        accel_local = self.state.acceleration_raw.copy()
        
        # Subtract calibration bias if calibrated
        if self.state.calibrated_position:
            accel_local -= self.state.bias["accel"]
        
        # Transform local acceleration to global frame
        accel_global = await self.transformer.rotate_vector(
            vector=accel_local,
            yaw=self.state.orientation[0],
            pitch=self.state.orientation[1],
            roll=self.state.orientation[2],
            invert=True
        )
        
        # If not calibrated, remove gravity (calibrated bias already includes gravity)
        if not self.state.calibrated_position:
            accel_global -= np.array([0.0, 0.0, GRAVITY])
        
        # Apply noise threshold - treat small accelerations as zero
        for i in range(3):
            if abs(accel_global[i]) < self.accel_threshold:
                accel_global[i] = 0.0
        
        # Store new acceleration for next iteration
        self.state.acceleration = accel_global
        
        if display:
            print(f"Position: {self.state.position}, Velocity: {self.state.velocity}, Acceleration: {self.state.acceleration}")
        
        return True
    
    async def update_positions_from_imu(self, **kwargs):
        """
        Update all sensor positions relative to the ground position.
        Ground position is already calculated in update_imu_position.
        """
        self.state.sensor_positions["lf_left"] = await self.transformer.translate_vector(
            vector=self.state.position,
            translation=await self.transformer.rotate_vector(
                vector=self.ground_to_lf_left,
                yaw=self.state.orientation[0],
                pitch=self.state.orientation[1],
                roll=self.state.orientation[2],
                invert=False
            )
        )

        self.state.sensor_positions["lf_right"] = await self.transformer.translate_vector(
            vector=self.state.position,
            translation=await self.transformer.rotate_vector(
                vector=self.ground_to_lf_right,
                yaw=self.state.orientation[0],
                pitch=self.state.orientation[1],
                roll=self.state.orientation[2],
                invert=False
            )
        )

        self.state.sensor_positions["color_sensor"] = await self.transformer.translate_vector(
            vector=self.state.position,
            translation=await self.transformer.rotate_vector(
                vector=self.ground_to_color,
                yaw=self.state.orientation[0],
                pitch=self.state.orientation[1],
                roll=self.state.orientation[2],
                invert=False
            )
        )

        self.state.sensor_positions["cargo_deploy"] = await self.transformer.translate_vector(
            vector=self.state.position,
            translation=await self.transformer.rotate_vector(
                vector=self.ground_to_cargo,
                yaw=self.state.orientation[0],
                pitch=self.state.orientation[1],
                roll=self.state.orientation[2],
                invert=False
            )
        )

    async def update_position(self, **kwargs):
        """
        Update position by integrating IMU data.
        
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
        Update the location state based on sensor data from State.
        Assumes SensorInput is running and updating State.

        Args:
            dt (float): Time step in seconds.
        """
        # Apply calibration bias to get corrected values (if calibrated)
        if self.state.calibrated_position and self.state.bias is not None and "accel" in self.state.bias:
            self.state.acceleration = self.state.acceleration_raw - self.state.bias["accel"]
        else:
            self.state.acceleration = self.state.acceleration_raw.copy()
        
        if self.state.calibrated_orientation and self.state.bias is not None and "gyro" in self.state.bias:
            self.state.angular_velocity = self.state.angular_velocity_raw - self.state.bias["gyro"]
        else:
            self.state.angular_velocity = self.state.angular_velocity_raw.copy()

        # Integrate acceleration to update velocity and position
        self.state.velocity = (1 - self.velocity_decay) * (self.state.velocity + self.state.acceleration * dt)
        self.state.position += self.state.velocity * dt

        # Update orientation using angular velocity
        self.state.orientation += self.state.angular_velocity * dt

        # Apply thresholds to filter noise
        if np.linalg.norm(self.state.acceleration) < self.accel_threshold:
            self.state.acceleration = np.zeros(3)

        if np.linalg.norm(self.state.velocity) < self.motor_velocity_threshold:
            self.state.velocity = np.zeros(3)


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
    from systems.sensors import SensorInput
    from systems.state import State
    
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