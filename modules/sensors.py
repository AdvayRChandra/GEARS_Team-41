"""
sensors.py

Centralized sensor management for MACRO.

This module provides:
- SensorInput: Unified interface for all hardware sensors

The SensorInput class manages all sensors (IMU, ultrasonic, line finders, etc.)
and provides a single point of access for navigation and mobility systems.
"""

import asyncio
import numpy as np
from basehat import IMUSensor, UltrasonicSensor, Button, LineFinder
from buildhat import ColorSensor
from .state import State


class SensorInput:
    """
    Centralized sensor management class.
    
    Provides unified access to all hardware sensors for navigation,
    mobility, and other systems.
    
    Args:
        imu (bool): Enable IMU sensor (default: True)
        ultrasonic_pin (int): GPIO pin for ultrasonic sensor (default: 26)
        ultrasonic (bool): Enable ultrasonic sensor (default: True)
        line_finder_left_pin (int): GPIO pin for left line finder (default: 16)
        line_finder_right_pin (int): GPIO pin for right line finder (default: 5)
        line_finders (bool): Enable line finder sensors (default: False)
        button_pin (int): GPIO pin for button (default: 22)
        button (bool): Enable button (default: False)
        color_sensor_port (str): Build HAT port for color sensor (default: "D")
        color_sensor (bool): Enable color sensor (default: False)
    
    Attributes:
        imu: IMUSensor instance or None
        ultrasonic: UltrasonicSensor instance or None
        line_finder_left: LineFinder instance or None
        line_finder_right: LineFinder instance or None
        button: Button instance or None
        color_sensor: ColorSensor instance or None
    
    Note:
        Magnetic field detection uses the IMU magnetometer via get_mag()
        and get_magnetic_magnitude() methods.
    """
    
    def __init__(self, **kwargs):
        # IMU sensor
        if kwargs.get("imu", True):
            self.imu = IMUSensor()
        else:
            self.imu = None
        
        # Ultrasonic sensor
        if kwargs.get("ultrasonic", True):
            ultrasonic_pin = kwargs.get("ultrasonic_pin", 26)
            self.ultrasonic = UltrasonicSensor(ultrasonic_pin)
        else:
            self.ultrasonic = None
        
        # Line finder sensors (disabled by default)
        if kwargs.get("line_finders", False):
            left_pin = kwargs.get("line_finder_left_pin", 16)
            right_pin = kwargs.get("line_finder_right_pin", 5)
            self.line_finder_left = LineFinder(left_pin)
            self.line_finder_right = LineFinder(right_pin)
        else:
            self.line_finder_left = None
            self.line_finder_right = None
        
        # Button (disabled by default)
        if kwargs.get("button", False):
            button_pin = kwargs.get("button_pin", 22)
            self.button_sensor = Button(button_pin)
        else:
            self.button_sensor = None
        
        # Color sensor (disabled by default)
        if kwargs.get("color_sensor", False):
            color_port = kwargs.get("color_sensor_port", "D")
            self.color_sensor = ColorSensor(color_port)
        else:
            self.color_sensor = None
        
        # Cached sensor values
        self._accel = np.array([0.0, 0.0, 0.0])
        self._gyro = np.array([0.0, 0.0, 0.0])
        self._mag = np.array([0.0, 0.0, 0.0])
        self._distance = 0.0
        
        # Centralized state
        self.state = kwargs.get("state", State())
    
    # -------------------------------------------------------------------------
    # IMU Methods
    # -------------------------------------------------------------------------
    
    async def get_accel(self):
        """
        Get acceleration from IMU.
        
        Returns:
            tuple: (ax, ay, az) in m/s²
        """
        if self.imu is None:
            return (0.0, 0.0, 0.0)
        
        self._accel = np.array(self.imu.getAccel())
        return tuple(self._accel)
    
    async def get_gyro(self):
        """
        Get angular velocity from IMU.
        
        Returns:
            tuple: (gx, gy, gz) in degrees/second
        """
        if self.imu is None:
            return (0.0, 0.0, 0.0)
        
        self._gyro = np.array(self.imu.getGyro())
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
        return float(np.linalg.norm(self._mag))
    
    # -------------------------------------------------------------------------
    # Ultrasonic Methods
    # -------------------------------------------------------------------------
    
    async def get_distance(self):
        """
        Get distance from ultrasonic sensor.
        
        Returns:
            float: Distance in centimeters
        """
        if self.ultrasonic is None:
            return -1.0
        
        try:
            self._distance = float(self.ultrasonic.getDist)
        except:
            self._distance = -1.0
        
        return self._distance
    
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
    # Color Sensor Methods
    # -------------------------------------------------------------------------
    
    async def get_color(self):
        """
        Get detected color name from color sensor.
        
        Returns:
            str: Color name (e.g., "black", "white", "red", etc.) or "none"
        """
        if self.color_sensor is None:
            return "none"
        
        try:
            return self.color_sensor.get_color()
        except:
            return "none"
    
    async def is_black(self):
        """
        Check if color sensor detects black.
        
        Returns:
            int: 1 if black detected, 0 otherwise
        """
        if self.color_sensor is None:
            return 0
        
        try:
            color = self.color_sensor.get_color()
            return 1 if color == "black" else 0
        except:
            return 0
    
    def has_color_sensor(self):
        """Check if color sensor is available."""
        return self.color_sensor is not None
    
    # -------------------------------------------------------------------------
    # Hall Sensor Methods (DEPRECATED - use IMU magnetometer instead)
    # -------------------------------------------------------------------------
    
    def get_hall_value(self):
        """
        DEPRECATED: Hall sensor removed. Use get_mag() or get_magnetic_magnitude() instead.
        
        Returns:
            bool: Always returns False
        """
        import warnings
        warnings.warn(
            "get_hall_value() is deprecated. Use get_mag() or get_magnetic_magnitude() instead.",
            DeprecationWarning,
            stacklevel=2
        )
        return False
    
    # -------------------------------------------------------------------------
    # Line Finder Methods
    # -------------------------------------------------------------------------
    
    async def get_line_left(self):
        """
        Get left line finder reading.
        
        Returns:
            bool: True if line detected
        """
        if self.line_finder_left is None:
            return False
        
        return self.line_finder_left.value
    
    async def get_line_right(self):
        """
        Get right line finder reading.
        
        Returns:
            bool: True if line detected
        """
        if self.line_finder_right is None:
            return False
        
        return self.line_finder_right.value
    
    # -------------------------------------------------------------------------
    # Utility Methods
    # -------------------------------------------------------------------------
    
    def has_imu(self):
        """Check if IMU is available."""
        return self.imu is not None
    
    def has_ultrasonic(self):
        """Check if ultrasonic sensor is available."""
        return self.ultrasonic is not None
    
    def has_button(self):
        """Check if button is available."""
        return self.button_sensor is not None
    
    def has_hall(self):
        """
        DEPRECATED: Hall sensor removed. Use has_imu() and get_mag() instead.
        
        Returns:
            bool: Always returns False
        """
        import warnings
        warnings.warn(
            "has_hall() is deprecated. Use has_imu() and get_mag() for magnetic sensing.",
            DeprecationWarning,
            stacklevel=2
        )
        return False
    
    def has_line_finders(self):
        """Check if line finders are available."""
        return self.line_finder_left is not None and self.line_finder_right is not None

    # -------------------------------------------------------------------------
    # State Update Methods
    # -------------------------------------------------------------------------
    
    async def update_state(self):
        """
        Update the State object with current sensor readings.
        """
        # Update IMU values
        if self.imu is not None:
            accel = self.imu.getAccel()
            self.state.sensors.acceleration_raw = np.array(accel)
            
            gyro = self.imu.getGyro()
            self.state.sensors.angular_velocity_raw = np.array(gyro)
            
            mag = self.imu.getMag()
            self.state.sensors.magnetic_field = float(np.linalg.norm(np.array(mag)))
        
        # Update ultrasonic distance
        if self.ultrasonic is not None:
            try:
                self.state.sensors.ultrasonic_distance = float(self.ultrasonic.getDist)
            except:
                pass  # Keep previous value on error
        
        # Update line finder values
        if self.line_finder_left is not None:
            self.state.sensors.lf_left_value = float(self.line_finder_left.value)
        if self.line_finder_right is not None:
            self.state.sensors.lf_right_value = float(self.line_finder_right.value)
        
        # Update button state
        if self.button_sensor is not None:
            self.state.sensors.button_pressed = self.button_sensor.is_pressed
        
        # Update color sensor
        if self.color_sensor is not None:
            try:
                color = self.color_sensor.get_color()
                # Map color to integer value
                color_map = {
                    "black": 0, "white": 1, "red": 2, "green": 3,
                    "blue": 4, "yellow": 5, "none": -1
                }
                self.state.sensors.color_sensor_value = color_map.get(color, -1)
            except:
                pass  # Keep previous value on error

    async def run_sensor_update(self, **kwargs):
        """
        Continuously update sensor readings to State at a fixed interval.
        
        Args:
            update_interval (float): Update interval in seconds (default: 0.05)
        """
        update_interval = kwargs.get("update_interval", 0.05)
        
        while True:
            await self.update_state()
            await asyncio.sleep(update_interval)
