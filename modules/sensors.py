"""
sensors.py

Centralized sensor management for MACRO.

This module provides:
- SensorInput: Unified interface for all hardware sensors

The SensorInput class manages all sensors (IMU gyroscope and magnetometer,
three ultrasonic sensors, button, IR sensor) and provides a single point of
access for navigation and mobility systems.
"""

import asyncio
import numpy as np
from basehat import IMUSensor, UltrasonicSensor, Button, IRSensor
from .state import State
from .config import RobotConfig


class SensorInput:
    """
    Centralized sensor management class.
    
    Provides unified access to all hardware sensors for navigation,
    mobility, and other systems.
    
    Args:
        state (State): Centralized state object shared with Location and Navigation.
        config (RobotConfig): Static hardware configuration (pins, enabled flags).
    
    Attributes:
        imu: IMUSensor instance or None
        ultrasonic_left: UltrasonicSensor instance or None
        ultrasonic_right: UltrasonicSensor instance or None
        ultrasonic_center: UltrasonicSensor instance or None
        button_sensor: Button instance or None
        ir_sensor: IRSensor instance or None
    """
    
    def __init__(self, state: State, config: RobotConfig):
        sc = config.sensors

        # IMU sensor (gyroscope and magnetometer)
        if sc.enable_imu:
            self.imu = IMUSensor()
        else:
            self.imu = None
        
        # Three ultrasonic sensors
        if sc.enable_ultrasonic:
            self.ultrasonic_left = UltrasonicSensor(sc.ultrasonic_left.pin)
            self.ultrasonic_right = UltrasonicSensor(sc.ultrasonic_right.pin)
            self.ultrasonic_center = UltrasonicSensor(sc.ultrasonic_center.pin)
        else:
            self.ultrasonic_left = None
            self.ultrasonic_right = None
            self.ultrasonic_center = None
        
        # Button (disabled by default)
        if sc.enable_button:
            self.button_sensor = Button(sc.button_pin)
        else:
            self.button_sensor = None

        # IR sensor (disabled by default)
        if sc.enable_ir_sensor:
            self.ir_sensor = IRSensor(sc.ir_sensor_left.pin, sc.ir_sensor_right.pin)
        else:
            self.ir_sensor = None
        
        # Cached sensor values
        self._gyro = np.array([0.0, 0.0, 0.0])
        self._mag = np.array([0.0, 0.0, 0.0])
        self._dist_left = -1.0
        self._dist_right = -1.0
        self._dist_center = -1.0
        self._ir_value1 = -1
        self._ir_value2 = -1
        
        # Centralized state
        self.state = state
    
    # -------------------------------------------------------------------------
    # IMU Methods
    # -------------------------------------------------------------------------
    
    async def get_gyro(self):
        """
        Get angular velocity from IMU.
        
        Returns:
            tuple: (gx, gy, gz) in degrees/second
        """
        if self.imu is None:
            return (0.0, 0.0, 0.0)

        self._gyro = np.array(self.imu.getGyro())
        self.state.sensors.angular_velocity_raw = self._gyro.copy()
        return tuple(self._gyro)
    
    async def get_mag(self):
        """
        Get magnetic field from IMU.
        
        Returns:
            tuple: (mx, my, mz) in micro-tesla
        """
        if self.imu is None:
            return (0.0, 0.0, 0.0)
        
        self._mag = np.array(self.imu.getMag())
        self.state.sensors.magnetic_field = float(np.linalg.norm(self._mag))
        return tuple(self._mag)
    
    async def get_magnetic_magnitude(self):
        """
        Get magnetic field magnitude.
        
        Returns:
            float: Magnitude in micro-tesla
        """
        if self.imu is None:
            return 0.0
        
        self._mag = np.array(self.imu.getMag())
        self.state.sensors.magnetic_field = float(np.linalg.norm(self._mag))
        return self.state.sensors.magnetic_field
    
    # -------------------------------------------------------------------------
    # Ultrasonic Methods
    # -------------------------------------------------------------------------
    
    async def get_distance_left(self):
        """
        Get distance from left ultrasonic sensor.
        
        Returns:
            float: Distance in centimeters, or -1.0 on error
        """
        if self.ultrasonic_left is None:
            return -1.0
        val = self.ultrasonic_left.getDist
        self._dist_left = float(val) if val is not None else -1.0
        self.state.sensors.ultrasonic_left.distance = self._dist_left
        return self._dist_left
    
    async def get_distance_right(self):
        """
        Get distance from right ultrasonic sensor.
        
        Returns:
            float: Distance in centimeters, or -1.0 on error
        """
        if self.ultrasonic_right is None:
            return -1.0
        val = self.ultrasonic_right.getDist
        self._dist_right = float(val) if val is not None else -1.0
        self.state.sensors.ultrasonic_right.distance = self._dist_right
        return self._dist_right
    
    async def get_distance_center(self):
        """
        Get distance from center ultrasonic sensor.
        
        Returns:
            float: Distance in centimeters, or -1.0 on error
        """
        if self.ultrasonic_center is None:
            return -1.0
        val = self.ultrasonic_center.getDist
        self._dist_center = float(val) if val is not None else -1.0
        self.state.sensors.ultrasonic_center.distance = self._dist_center
        return self._dist_center
    
    # -------------------------------------------------------------------------
    # Button Methods
    # -------------------------------------------------------------------------
    
    async def is_button_pressed(self):
        """
        Check if button is pressed.
        
        Returns:
            bool: True if pressed, False otherwise
        """
        if self.button_sensor is None:
            return False
        
        return self.button_sensor.is_pressed

    # -------------------------------------------------------------------------
    # IR Sensor Methods
    # -------------------------------------------------------------------------

    async def get_ir_values(self):
        """
        Get readings from both IR sensor elements.

        Returns:
            tuple: (value1, value2) each in range 0-255, or (-1, -1) if unavailable
        """
        if self.ir_sensor is None:
            return (-1, -1)

        self._ir_value1 = self.ir_sensor.value1
        self._ir_value2 = self.ir_sensor.value2
        self.state.sensors.ir_sensor_left.value = self._ir_value1
        self.state.sensors.ir_sensor_right.value = self._ir_value2
        return (self._ir_value1, self._ir_value2)

    # -------------------------------------------------------------------------
    # Utility Methods
    # -------------------------------------------------------------------------
    
    def has_imu(self):
        """Check if IMU is available."""
        return self.imu is not None
    
    def has_ultrasonic(self):
        """Check if all three ultrasonic sensors are available."""
        return (
            self.ultrasonic_left is not None
            and self.ultrasonic_right is not None
            and self.ultrasonic_center is not None
        )
    
    def has_button(self):
        """Check if button is available."""
        return self.button_sensor is not None

    def has_ir_sensor(self):
        """Check if IR sensor is available."""
        return self.ir_sensor is not None

    # -------------------------------------------------------------------------
    # State Update Methods
    # -------------------------------------------------------------------------
    
    async def update_state(self):
        """
        Update the State object with current sensor readings.
        """
        # Update IMU gyroscope and magnetometer values
        if self.imu is not None:
            gyro = self.imu.getGyro()
            self.state.sensors.angular_velocity_raw = np.array(gyro)
            
            mag = self.imu.getMag()
            self.state.sensors.magnetic_field = float(np.linalg.norm(np.array(mag)))
        
        # Update ultrasonic distances
        if self.ultrasonic_left is not None:
            val = self.ultrasonic_left.getDist
            self.state.sensors.ultrasonic_left.distance = float(val) if val is not None else -1.0
        if self.ultrasonic_right is not None:
            val = self.ultrasonic_right.getDist
            self.state.sensors.ultrasonic_right.distance = float(val) if val is not None else -1.0
        if self.ultrasonic_center is not None:
            val = self.ultrasonic_center.getDist
            self.state.sensors.ultrasonic_center.distance = float(val) if val is not None else -1.0
        
        # Update button state
        if self.button_sensor is not None:
            self.state.sensors.button_pressed = self.button_sensor.is_pressed

        # Update IR sensor values
        if self.ir_sensor is not None:
            self.state.sensors.ir_sensor_left.value = self.ir_sensor.value1
            self.state.sensors.ir_sensor_right.value = self.ir_sensor.value2

    async def setup(self):
        """
        Prime State with an initial sensor reading before the update loop starts.
        """
        await self.update_state()

    async def run_sensor_update(self, **kwargs):
        """
        Continuously update sensor readings to State at a fixed interval.

        Call setup() before starting this loop so State is valid on the first
        tick consumed by Location and Navigation.
        
        Args:
            update_interval (float): Update interval in seconds (default: 0.05)
        """
        update_interval = kwargs.get("update_interval", 0.05)
        
        while True:
            await self.update_state()
            await asyncio.sleep(update_interval)
