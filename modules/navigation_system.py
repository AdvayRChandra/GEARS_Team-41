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
    Navigation: Waypoint navigation and obstacle detection with timestamped logging
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

        await self._compute_sensor_poses()

        return True

    def _euler_from_matrix(self, R: np.ndarray) -> np.ndarray:
        """
        Extract [yaw, pitch, roll] from a ZYX rotation matrix built by Transformation.
        Handles the gimbal-lock singularity (|pitch| == 90°) by setting roll = 0.
        """
        sin_pitch = np.clip(-R[0, 2], -1.0, 1.0)
        pitch = math.asin(sin_pitch)

        if abs(abs(sin_pitch) - 1.0) < 1e-6:   # gimbal lock
            yaw = math.atan2(-R[1, 0], R[1, 1])
            roll = 0.0
        else:
            yaw = math.atan2(R[0, 1], R[0, 0])
            roll = math.atan2(R[1, 2], R[2, 2])

        if self.mode == "degrees":
            return np.array([math.degrees(yaw), math.degrees(pitch), math.degrees(roll)])
        return np.array([yaw, pitch, roll])

    async def _compute_sensor_poses(self):
        """
        Compute world-frame pose for each ultrasonic sensor and world-frame
        position for the IR sensor from the robot's current pose and the
        sensor-to-IMU mount transforms stored in SensorState.

        For each ultrasonic sensor:
          1. Position transform:
             world_pos = robot_position + R_robot @ local_position
          2. Orientation transform:
             R_world = R_robot @ R_sensor_local
             world_orient = euler(R_world)

        For the IR sensor (no orientation):
          1. Position transform only:
             world_pos = robot_position + R_robot @ local_position
        """
        R_robot = await self.transformer.get_rotation(orientation=self.state.nav.orientation)

        for side in ("left", "right", "center"):
            sensor = getattr(self.state.sensors, f"ultrasonic_{side}")

            # Step 1 — position transform
            world_pos = self.state.nav.position + np.matmul(R_robot, sensor.local_position)

            # Step 2 — orientation transform
            R_sensor = await self.transformer.get_rotation(orientation=sensor.local_orientation)
            R_world = np.matmul(R_robot, R_sensor)
            world_orient = self._euler_from_matrix(R_world)

            sensor.world_position = world_pos
            sensor.world_orientation = world_orient

        # IR sensor — position only, no orientation
        ir = self.state.sensors.ir_sensor
        ir.world_position = self.state.nav.position + np.matmul(R_robot, ir.local_position)
    
    async def update(self, dt: float = 0.1):
        """Update both orientation and position."""
        await self.update_orientation(dt=dt)
        await self.update_position(dt=dt)
        return True

    async def setup(self):
        """
        Snapshot the current motor positions as the odometry baseline.

        Call this once after SensorInput.setup() so the first update() delta
        starts from the actual motor positions rather than the constructor-time
        values (which may differ if motors moved during hardware init).
        """
        self._prev_motor_left = self.state.motor_left.position
        self._prev_motor_right = self.state.motor_right.position

    async def run_location_update(self, **kwargs):
        """
        Continuously update position and orientation at a fixed interval.

        Reads sensor data written by SensorInput into state.sensors and writes
        the computed pose into state.nav.  Run concurrently with
        SensorInput.run_sensor_update() so state.sensors is always fresh.

        Args:
            update_interval (float): Update interval in seconds (default: 0.05)
        """
        update_interval = kwargs.get("update_interval", 0.05)
        loop = asyncio.get_running_loop()
        last_time = loop.time()

        while True:
            await asyncio.sleep(update_interval)
            now = loop.time()
            dt = now - last_time
            last_time = now
            await self.update(dt=dt)


class Navigation:
    """
    Waypoint navigation and obstacle detection on top of a shared State.

    Reads pose data written by Location and sensor data written by SensorInput
    to project ultrasonic readings into world-frame obstacle positions and to
    navigate to 2D waypoints.  Does not own a Location; both classes share the
    same State instance.

    Args:
        state (State): Centralized state object shared with SensorInput and Location
        motion (MotionController): Motion controller used to issue drive commands
        angle_tolerance (float): Heading error in degrees considered "aligned" (default: 5.0)

    Attributes:
        _obstacles (dict): Latest world-frame obstacle positions keyed by "left", "right", "center"
        _log (list): Timestamped position/orientation history
    """
    def __init__(self, state: State, motion: MotionController, **kwargs):
        self.state = state
        self.motion = motion
        self.angle_tolerance = kwargs.get("angle_tolerance", 5.0)
        self._log = []
        self._obstacles = {"left": None, "right": None, "center": None}
        self.transformer = Transformation(mode=state.mode)

    async def get_obstacle_positions(self) -> dict:
        """
        Project each ultrasonic reading into the global frame.

        For each sensor with a valid distance reading, computes the world-frame
        position of the detected surface:

            obstacle = sensor.world_position + R(sensor.world_orientation) @ [distance, 0, 0]

        Sensors reporting distance == -1.0 are skipped (no detection / timeout).

        Returns:
            dict with keys "left", "right", "center". Each value is either a
            np.ndarray [x, y, z] in meters (global frame) or None if that sensor
            had no valid reading.
        """
        result = {}
        for side in ("left", "right", "center"):
            sensor = getattr(self.state.sensors, f"ultrasonic_{side}")
            if sensor.distance < 0.0:
                result[side] = None
                continue

            # Sensor faces along its local +X axis; distance is in centimeters.
            distance_m = sensor.distance / 100.0
            R_sensor = await self.transformer.get_rotation(orientation=sensor.world_orientation)
            facing = np.matmul(R_sensor, np.array([1.0, 0.0, 0.0]))
            result[side] = sensor.world_position + facing * distance_m

        return result

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

    async def setup(self):
        """
        Seed the navigation log with the robot's initial pose.

        Call this once after Location.setup() so the log timestamp is anchored
        to the moment navigation becomes active rather than object construction.
        """
        loop = asyncio.get_running_loop()
        self._log.append({
            "time": loop.time(),
            "position": self.state.nav.position.copy(),
            "orientation": self.state.nav.orientation.copy(),
        })

    async def run_navigation_update(self, **kwargs):
        """
        Continuously log navigation state and refresh obstacle positions.

        Reads state.nav (written by Location.run_location_update) and
        state.sensors (written by SensorInput.run_sensor_update) each tick.
        Stores the latest obstacle world-frame positions in self._obstacles so
        other systems can query them without calling get_obstacle_positions()
        themselves.

        Args:
            update_interval (float): Update interval in seconds (default: 0.1)
        """
        update_interval = kwargs.get("update_interval", 0.1)
        loop = asyncio.get_running_loop()

        while True:
            await asyncio.sleep(update_interval)
            self._obstacles = await self.get_obstacle_positions()
            self._log.append({
                "time": loop.time(),
                "position": self.state.nav.position.copy(),
                "orientation": self.state.nav.orientation.copy(),
            })


if __name__ == "__main__":
    # Run from the project root as:  python -m modules.navigation_system
    # (relative imports at the top of this file require the package context)
    from .sensors import SensorInput

    async def _display_loop(state: State, interval: float = 0.5):
        while True:
            await asyncio.sleep(interval)
            s = state.sensors
            n = state.nav
            print(
                "\n--- State ---\n"
                f"  position    : {n.position}\n"
                f"  velocity    : {n.velocity}\n"
                f"  orientation : {n.orientation}\n"
                f"  gyro (raw)  : {s.angular_velocity_raw}\n"
                f"  dist left   : {s.ultrasonic_left.distance:.1f} cm\n"
                f"  dist right  : {s.ultrasonic_right.distance:.1f} cm\n"
                f"  dist center : {s.ultrasonic_center.distance:.1f} cm\n"
                f"  motor left  : pos={state.motor_left.position:.1f}  moving={state.motor_left.is_moving}\n"
                f"  motor right : pos={state.motor_right.position:.1f}  moving={state.motor_right.is_moving}"
            )

    async def main():
        state = State()

        sensors = SensorInput(state=state)
        location = Location(state=state)
        motion = MotionController(state=state)   # required by Navigation; no motion commands issued
        navigation = Navigation(state=state, motion=motion)

        await sensors.setup()
        await location.setup()
        await motion.setup()
        await navigation.setup()

        tasks = [
            asyncio.create_task(sensors.run_sensor_update()),
            asyncio.create_task(location.run_location_update()),
            asyncio.create_task(motion.run_motor_update()),
            asyncio.create_task(navigation.run_navigation_update()),
            asyncio.create_task(_display_loop(state)),
        ]

        print("Running — press Ctrl+C to stop.\n")
        try:
            await asyncio.gather(*tasks)
        except (KeyboardInterrupt, asyncio.CancelledError):
            pass
        finally:
            for task in tasks:
                task.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)
            print("\nShutdown complete.")

    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass