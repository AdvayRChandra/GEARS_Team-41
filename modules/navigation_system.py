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
import csv
import glob
import os
import numpy as np
import math
from .state import State
from .motion import MotionController
from .config import RobotConfig


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
            [math.cos(yaw), -math.sin(yaw), 0],
            [math.sin(yaw),  math.cos(yaw), 0],
            [0,              0,             1]
        ])
        return np.transpose(R_yaw) if invert else R_yaw
    
    async def get_rotation_pitch(self, **kwargs):
        """Generate rotation matrix for pitch (Y-axis rotation)."""
        pitch = kwargs.get("pitch", 0.0)
        invert = kwargs.get("invert", False)
        
        if self.mode == "degrees":
            pitch = math.radians(pitch)

        R_pitch = np.array([
            [ math.cos(pitch), 0, math.sin(pitch)],
            [0,                1, 0              ],
            [-math.sin(pitch), 0, math.cos(pitch)]
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
            [0, math.cos(roll), -math.sin(roll) ],
            [0, math.sin(roll),  math.cos(roll) ]
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
        state (State): Centralized state object for sensor and navigation data.
        config (RobotConfig): Static robot configuration (wheel diameter, mode,
            initial pose, sensor mount transforms).
    
    Attributes:
        state.nav.position: Current position [x, y, z] in global frame
        state.nav.velocity: Current velocity [vx, vy, vz] in global frame
        state.nav.orientation: Current orientation [yaw, pitch, roll]
    """
    
    def __init__(self, state: State, config: RobotConfig):
        self.state = state
        self.config = config
        self.wheel_diameter = config.wheel_diameter
        self.mode = config.mode

        # Seed nav state from config
        self.state.nav.position = config.initial_position.copy()
        self.state.nav.velocity = np.zeros(3)
        self.state.nav.acceleration = np.zeros(3)
        orientation = config.initial_orientation.copy()
        orientation[0] = config.initial_heading
        self.state.nav.orientation = orientation
        self.state.nav.angular_velocity = np.zeros(3)
        self.state.nav.angular_acceleration = np.zeros(3)

        # Track previous motor positions to compute deltas
        self._prev_motor_left = state.motor_left.position
        self._prev_motor_right = state.motor_right.position

        self.transformer = Transformation(mode=self.mode)
    
    _DISCRETE_DIRS = [[1, 0], [0, 1], [-1, 0], [0, -1]]

    def _compute_discrete_orientation(self, yaw) -> list:
        """Snap a continuous yaw angle to the nearest cardinal direction.

        Returns one of: [1,0] (+X), [0,1] (+Y), [-1,0] (-X), [0,-1] (-Y).
        """
        yaw_deg = yaw if self.mode == "degrees" else math.degrees(yaw)
        idx = round((yaw_deg % 360) / 90) % 4
        return self._DISCRETE_DIRS[idx]

    async def update_orientation(self, dt: float = 0.1):
        gyro_raw = self.state.sensors.angular_velocity_raw
        gyro = await self.transformer.rotate_vector(
            vector=gyro_raw,
            orientation=self.config.sensors.imu.local_orientation
        )

        self.state.nav.orientation += (
            0.5 * self.state.nav.angular_acceleration * dt ** 2
            + self.state.nav.angular_velocity * dt
        )
        self.state.nav.angular_acceleration = (gyro - self.state.nav.angular_velocity) / dt
        self.state.nav.angular_velocity = gyro

        self.state.nav.discrete_orientation = self._compute_discrete_orientation(
            self.state.nav.orientation[0]
        )

        return True
    
    async def update_position(self, dt: float = 0.1):
        delta_left = self.state.motor_left.position - self._prev_motor_left
        delta_right = self.state.motor_right.position - self._prev_motor_right
        delta_degrees = (delta_left - delta_right) / 2
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
        sin_pitch = np.clip(-R[2, 0], -1.0, 1.0)
        pitch = math.asin(sin_pitch)

        if abs(abs(sin_pitch) - 1.0) < 1e-6:   # gimbal lock
            yaw = math.atan2(-R[0, 1], R[1, 1])
            roll = 0.0
        else:
            yaw = math.atan2(R[1, 0], R[0, 0])
            roll = math.atan2(R[2, 1], R[2, 2])

        if self.mode == "degrees":
            return np.array([math.degrees(yaw), math.degrees(pitch), math.degrees(roll)])
        return np.array([yaw, pitch, roll])

    async def _compute_sensor_poses(self):
        """
        Compute world-frame pose for each ultrasonic sensor and world-frame
        position for the IR sensor from the robot's current pose and the
        sensor-to-IMU mount transforms stored in RobotConfig.

        For each ultrasonic sensor:
          1. Position transform:
             world_pos = robot_position + R_robot @ local_position
          2. Orientation transform:
             R_world = R_robot @ R_sensor_local
             world_orient = euler(R_world)

        For the IR sensor (no orientation):
          1. Position transform only:
             world_pos = robot_position + R_robot @ local_position

        For the IMU magnetic sensor (no orientation):
          1. Position transform only:
             world_pos = robot_position + R_robot @ local_position
        """
        R_robot = await self.transformer.get_rotation(orientation=self.state.nav.orientation)

        for side in ("left", "right", "center"):
            sensor_state = getattr(self.state.sensors, f"ultrasonic_{side}")
            sensor_cfg = getattr(self.config.sensors, f"ultrasonic_{side}")

            # Step 1 — position transform
            world_pos = self.state.nav.position + np.matmul(R_robot, sensor_cfg.local_position)

            # Step 2 — orientation transform
            R_sensor = await self.transformer.get_rotation(orientation=sensor_cfg.local_orientation)
            R_world = np.matmul(R_robot, R_sensor)
            world_orient = self._euler_from_matrix(R_world)

            sensor_state.world_position = world_pos
            sensor_state.world_orientation = world_orient

        # IR sensor left and right elements — position only, no orientation
        self.state.sensors.ir_sensor_left.world_position = (
            self.state.nav.position + np.matmul(R_robot, self.config.sensors.ir_sensor_left.local_position)
        )
        self.state.sensors.ir_sensor_right.world_position = (
            self.state.nav.position + np.matmul(R_robot, self.config.sensors.ir_sensor_right.local_position)
        )

        # IMU magnetic sensor — position only (orientation applied to gyro in update_orientation)
        self.state.sensors.mag_world_position = (
            self.state.nav.position + np.matmul(R_robot, self.config.sensors.imu.local_position)
        )
    
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
        state (State): Centralized state object shared with SensorInput and Location.
        motion (MotionController): Motion controller used to issue drive commands.
        config (RobotConfig): Static robot configuration (mode, angle_tolerance).

    Attributes:
        _obstacles (dict): Latest world-frame obstacle positions keyed by "left", "right", "center"
        _log (list): Timestamped position/orientation history
    """
    def __init__(self, state: State, motion: MotionController, config: RobotConfig):
        self.state = state
        self.motion = motion
        self.angle_tolerance = config.angle_tolerance
        self._log = []
        self._obstacles = {"left": None, "right": None, "center": None}
        self.transformer = Transformation(mode=config.mode)
        self.map = Map(state=state, config=config)

    async def get_obstacle_positions(self) -> dict:
        """
        Project each ultrasonic reading into the global frame using the robot's
        discrete orientation so obstacle positions are snapped to cardinal
        directions on the map grid.

        The robot's discrete heading (state.nav.discrete_orientation) defines
        the forward cardinal direction [dx, dy].  Each sensor's facing vector
        is derived from that:
            center → forward:        [ dx,  dy, 0]
            left   → CCW 90° turn:   [-dy,  dx, 0]
            right  → CW  90° turn:   [ dy, -dx, 0]

        Sensors reporting distance == -1.0 are skipped (no detection / timeout).

        Returns:
            dict with keys "left", "right", "center". Each value is either a
            np.ndarray [x, y, z] in meters (global frame) or None if that sensor
            had no valid reading.
        """
        dx, dy = self.state.nav.discrete_orientation
        discrete_facings = {
            "center": np.array([ dx,  dy, 0.0]),
            "left":   np.array([-dy,  dx, 0.0]),
            "right":  np.array([ dy, -dx, 0.0]),
        }

        result = {}
        for side in ("left", "right", "center"):
            sensor = getattr(self.state.sensors, f"ultrasonic_{side}")
            if sensor.distance < 0.0:
                result[side] = None
                continue

            # Sensor faces along its discrete cardinal direction; distance is in centimeters.
            distance_m = sensor.distance / 100.0
            result[side] = sensor.world_position + discrete_facings[side] * distance_m

        return result

    async def go_to(self, destination_cell):
        """
        Navigate to a destination grid cell using automatic_turn for obstacle-aware routing.

        Marks the destination cell as value 4 (exit) on the map, then starts an
        asynchronous navigation loop that repeatedly calls automatic_turn().  The
        loop is cancelled and the robot is stopped once the robot's current grid
        cell matches the destination cell.

        Args:
            destination_cell: Target grid cell as (col, row) indices.
        """
        dest_col, dest_row = int(destination_cell[0]), int(destination_cell[1])

        # Mark the destination on the map
        if 0 <= dest_col < self.map.grid_width and 0 <= dest_row < self.map.grid_height:
            self.map.grid[dest_row, dest_col] = 4

        async def _navigate_loop():
            while True:
                await self.automatic_turn()
                await asyncio.sleep(0)

        nav_task = asyncio.ensure_future(_navigate_loop())

        try:
            while True:
                cur_col, cur_row = self.map._world_to_grid(
                    self.state.nav.position[0], self.state.nav.position[1]
                )
                if cur_col == dest_col and cur_row == dest_row:
                    nav_task.cancel()
                    self.motion.stop()
                    break
                await asyncio.sleep(0)
        except asyncio.CancelledError:
            nav_task.cancel()
            self.motion.stop()
            raise

    async def turn_degrees(self, degrees: float):
        """
        Turn the robot by a given number of degrees based on current orientation.

        Uses turn_left for negative degrees and turn_right for positive degrees.
        Stops once the target yaw is reached within angle_tolerance.

        Args:
            degrees (float): Degrees to turn. Positive = right, negative = left.
        """
        target_yaw = self.state.nav.orientation[0] + degrees

        while True:
            current_yaw = self.state.nav.orientation[0]
            angle_error = (target_yaw - current_yaw + 180) % 360 - 180

            if abs(angle_error) <= self.angle_tolerance:
                self.motion.stop()
                break

            if angle_error < 0:
                self.motion.turn_right()
            else:
                self.motion.turn_left()

            await asyncio.sleep(0)

    def determine_turn(self) -> int:
        """
        Determine the turn required based on current obstacle_neighbors and map cell values.

        The preferred direction is chosen from state.nav.obstacle_neighbors using
        map cell values:
            Priority 1 — cells with value 0 (unknown/free space)
            Priority 2 — cells with value 1 (previously travelled path)
            Direction preference within each priority tier: front > right > left > back

        Returns:
            int: Direction code.
                  0 = forward,  1 = right,  -1 = left,  2 = back.
                  Returns 0 if no preferred direction can be determined.
        """
        front, left, back, right = self.state.nav.obstacle_neighbors  # [0]=front [1]=left [2]=back [3]=right

        # Map-value priority navigation: front > right > left > back
        dir_preference = [
            (front,  0),
            (right,  1),
            (left,  -1),
            (back,   2),
        ]
        for target_val in (0, 1):
            for cell_val, direction in dir_preference:
                if cell_val == target_val:
                    return direction

        return 0

    async def automatic_turn(self):
        """
        Determine the required turn and execute it, or drive forward if none needed.

        Calls determine_turn() to get a direction code and maps it to degrees:
            0  → forward (no turn)
            1  → turn right  (+90°)
           -1  → turn left   (-90°)
            2  → turn around (180°)
        """
        _DIRECTION_DEGREES = {0: 0, 1: -90, -1: 90, 2: 180}
        _DIRECTION_LABELS  = {0: "forward", 1: "right", -1: "left", 2: "around"}
        direction = self.determine_turn()
        degrees = _DIRECTION_DEGREES[direction]
        if degrees != 0:
            print(f"[Navigation] Turning {_DIRECTION_LABELS[direction]} ({degrees:+d}°)")
            await self.turn_degrees(degrees)
        else:
            self.motion.forward()

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

            # Only map obstacles when the robot is settled on a cardinal heading.
            # Compare continuous yaw to the nearest 90° multiple; skip if the
            # error exceeds angle_tolerance to avoid recording mid-turn readings.
            yaw_deg = self.state.nav.orientation[0]
            nearest_cardinal = round(yaw_deg / 90) * 90
            heading_error = abs((yaw_deg - nearest_cardinal + 180) % 360 - 180)
            if heading_error <= self.angle_tolerance:
                self._obstacles = await self.get_obstacle_positions()
                self.map.update_obstacles(self._obstacles)

            self.map.update_path()
            self.state.nav.obstacle_neighbors = self.map.get_obstacle_states()
            self._log.append({
                "time": loop.time(),
                "position": self.state.nav.position.copy(),
                "orientation": self.state.nav.orientation.copy(),
            })

class Map:
    def __init__(self, state: State, config: RobotConfig):
        map_cfg = config.map
        self.resolution: float = map_cfg.resolution
        self.magnetic_threshold: float = map_cfg.magnetic_threshold
        self.ir_threshold: int = map_cfg.ir_threshold
        self.state: State = state
        self.config = config
        self.x_min: float = map_cfg.x_min
        self.y_max: float = map_cfg.y_max
        self.grid_width = int((map_cfg.x_max - map_cfg.x_min) / self.resolution)
        self.grid_height = int((map_cfg.y_max - map_cfg.y_min) / self.resolution)
        self.origin = (
            int(-map_cfg.x_min / self.resolution),
            int(map_cfg.y_max / self.resolution),
        )

        """
        2D occupancy grid representing the environment. Each cell can be:
        - 0: Unknown/Free space (not part of path)
        - 1: Path travelled (free space)
        - 2: Heat source (obstacle)
        - 3: Magnetic source (obstacle)
        - 4: Exit point (goal)
        - 5: Origin (starting position)
        - 6: Wall (detected by sensors but not yet classified as obstacle)
        """
        self.grid = np.zeros((self.grid_height, self.grid_width), dtype=np.uint8)
        self.grid[self.origin[1], self.origin[0]] = 5  # Mark the origin (starting position)

    def _world_to_grid(self, world_x: float, world_y: float):
        """Convert world-frame coordinates (meters) to grid indices.

        X maps to column: increasing world x → increasing column (left to right).
        Y maps to row: increasing world y → decreasing row (bottom to top),
        because row 0 is the top of the array.
        """
        grid_x = int((world_x - self.x_min) / self.resolution + 1e-9)
        grid_y = int((self.y_max - world_y) / self.resolution + 1e-9)
        return grid_x, grid_y

    def update_path(self):
        """Mark the robot's current world position as travelled path (1)."""
        pos = self.state.nav.position
        grid_x, grid_y = self._world_to_grid(pos[0], pos[1])
        if 0 <= grid_x < self.grid_width and 0 <= grid_y < self.grid_height:
            if self.grid[grid_y, grid_x] == 0:  # Only overwrite unknown cells
                self.grid[grid_y, grid_x] = 1

    def update_obstacles(self, obstacles: dict):
        """
        Update the grid with obstacle detections.

        Overwrite rules (by priority, highest first):
          - 4 (destination) and 5 (origin): never overridden by anything.
          - 1 (path): never overridden by any detection.
          - 2 (heat) / 3 (magnetic): override walls (6) and unknown (0).
          - 6 (wall): written only onto unknown cells (0).

        Ultrasonic obstacles use grid arithmetic rather than continuous world
        coordinates: the robot's current grid cell is used as the origin and
        the discrete heading is used to step exactly N cells to the obstacle.

        Grid direction mapping from world discrete heading [dx, dy]:
            center → (+dx, -dy)    (forward)
            left   → (-dy, -dx)    (CCW 90°)
            right  → (+dy, +dx)    (CW 90°)
        The sign flip on dy arises because grid rows increase downward while
        world Y increases upward.

        IR and magnetic sensors are placed at their continuous world positions
        (no directional projection needed).

        Args:
            obstacles: Dict with keys "left", "right", "center" — used only as
                a None sentinel (None means no valid reading for that side).
        """
        _PROTECTED = {1, 4, 5}  # path, destination, origin — never overwritten

        # Ultrasonic obstacles — grid arithmetic from robot cell + discrete direction
        dx, dy = self.state.nav.discrete_orientation
        robot_gx, robot_gy = self._world_to_grid(
            self.state.nav.position[0], self.state.nav.position[1]
        )
        side_grid_dirs = {
            "center": ( dx, -dy),
            "left":   (-dy, -dx),
            "right":  ( dy,  dx),
        }
        for side in ("left", "right", "center"):
            if obstacles.get(side) is None:
                continue
            sensor = getattr(self.state.sensors, f"ultrasonic_{side}")
            if sensor.distance < 0.0:
                continue
            cells = round((sensor.distance / 100.0) / self.resolution)
            gcol, grow = side_grid_dirs[side]
            grid_x = robot_gx + gcol * cells
            grid_y = robot_gy + grow * cells
            if 0 <= grid_x < self.grid_width and 0 <= grid_y < self.grid_height:
                if self.grid[grid_y, grid_x] == 0:  # Wall only onto unknown cells
                    self.grid[grid_y, grid_x] = 6

        # IR sensor — mark heat source; overrides walls but not path/origin/destination
        ir_left = self.state.sensors.ir_sensor_left
        ir_right = self.state.sensors.ir_sensor_right
        if ir_left.value >= self.ir_threshold:
            grid_x, grid_y = self._world_to_grid(ir_left.world_position[0], ir_left.world_position[1])
            if 0 <= grid_x < self.grid_width and 0 <= grid_y < self.grid_height:
                if self.grid[grid_y, grid_x] not in _PROTECTED:
                    self.grid[grid_y, grid_x] = 2  # Heat source
        if ir_right.value >= self.ir_threshold:
            grid_x, grid_y = self._world_to_grid(ir_right.world_position[0], ir_right.world_position[1])
            if 0 <= grid_x < self.grid_width and 0 <= grid_y < self.grid_height:
                if self.grid[grid_y, grid_x] not in _PROTECTED:
                    self.grid[grid_y, grid_x] = 2  # Heat source

        # Magnetic source — overrides walls but not path/origin/destination
        magnetic_field = self.state.sensors.magnetic_field
        if magnetic_field >= self.magnetic_threshold:
            mag_pos = self.state.sensors.mag_world_position
            grid_x, grid_y = self._world_to_grid(mag_pos[0], mag_pos[1])
            if 0 <= grid_x < self.grid_width and 0 <= grid_y < self.grid_height:
                if self.grid[grid_y, grid_x] not in _PROTECTED:
                    self.grid[grid_y, grid_x] = 3  # Magnetic source
    
    def get_obstacle_states(self) -> list:
        """Return the raw grid cell values for the 4 cells adjacent to the robot.

        Order: [front, left, back, right] relative to the robot's current
        discrete heading.  Uses the same grid-direction mapping as
        update_obstacles:
            front → ( dx, -dy)
            left  → (-dy, -dx)
            back  → (-dx,  dy)
            right → ( dy,  dx)

        Returns:
            list[int] of length 4.  Each element is the raw grid cell value
            (0=unknown/free, 1=path, 2=heat, 3=magnetic, 4=exit, 5=origin,
            6=wall). -1 indicates the adjacent cell is out of bounds.
        """
        dx, dy = self.state.nav.discrete_orientation
        robot_gx, robot_gy = self._world_to_grid(
            self.state.nav.position[0], self.state.nav.position[1]
        )
        directions = [
            ( dx, -dy),   # front
            (-dy, -dx),   # left
            (-dx,  dy),   # back
            ( dy,  dx),   # right
        ]
        result = []
        for gcol, grow in directions:
            nx, ny = robot_gx + gcol, robot_gy + grow
            if 0 <= nx < self.grid_width and 0 <= ny < self.grid_height:
                result.append(int(self.grid[ny, nx]))
            else:
                result.append(-1)
        return result

    def save_map(self, notes: str = "") -> str:
        """Save the current grid map to maps/team{team}_map{map_id}.csv.

        The maps/ folder is created if it does not exist. The map_id is assigned
        sequentially based on existing files already in the folder.

        Args:
            notes: Optional freeform notes written to the CSV header (default: "").

        Returns:
            The path to the saved file.
        """
        map_cfg = self.config.map
        team = map_cfg.team
        unit_length = map_cfg.unit_length
        unit = map_cfg.unit
        origin_str = f"({self.origin[0]},{self.origin[1]})"

        os.makedirs("maps", exist_ok=True)
        existing = glob.glob(f"maps/team{team}_map*.csv")
        if existing:
            ids = []
            prefix = f"team{team}_map"
            for p in existing:
                base = os.path.splitext(os.path.basename(p))[0]
                if base.startswith(prefix):
                    try:
                        ids.append(int(base[len(prefix):]))
                    except ValueError:
                        pass
            map_id = max(ids) + 1 if ids else 0
        else:
            map_id = 0

        filename = f"maps/team{team}_map{map_id}.csv"
        with open(filename, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([f"Team: {team}"])
            writer.writerow([f"Map: {map_id}"])
            writer.writerow([f"Unit Length: {unit_length}"])
            writer.writerow([f"Unit: {unit}"])
            writer.writerow([f"Origin: {origin_str}"])
            writer.writerow([f"Notes: {notes}"])
            export_grid = np.where(self.grid == 6, 0, self.grid)
            for row in export_grid:
                writer.writerow(row.tolist())

        return filename


if __name__ == "__main__":
    # Run from the project root as:  python -m modules.navigation_system
    # (relative imports at the top of this file require the package context)
    from .sensors import SensorInput
    from .config import RobotConfig

    async def _display_loop(state: State, map: 'Map', interval: float = 0.5):
        _CELL_NAMES = {0: "free", 1: "path", 2: "heat", 3: "mag", 4: "exit", 5: "origin", 6: "wall", -1: "OOB"}
        while True:
            await asyncio.sleep(interval)
            n = state.nav
            grid_x, grid_y = map._world_to_grid(n.position[0], n.position[1])
            front, left, back, right = n.obstacle_neighbors
            print(
                f"[Nav] cell=({grid_x},{grid_y})  "
                f"front={_CELL_NAMES.get(front, front)}  "
                f"right={_CELL_NAMES.get(right, right)}  "
                f"left={_CELL_NAMES.get(left, left)}  "
                f"back={_CELL_NAMES.get(back, back)}"
            )

    async def main(dest_cell):
        global navigation
        state = State()
        config = RobotConfig()

        sensors = SensorInput(state=state, config=config)
        location = Location(state=state, config=config)
        motion = MotionController(state=state, config=config)
        navigation = Navigation(state=state, motion=motion, config=config)

        await sensors.setup()
        await location.setup()
        await motion.setup()
        await navigation.setup()

        background_tasks = [
            asyncio.create_task(sensors.run_sensor_update()),
            asyncio.create_task(location.run_location_update()),
            asyncio.create_task(motion.run_motor_update()),
            asyncio.create_task(navigation.run_navigation_update()),
            asyncio.create_task(_display_loop(state, navigation.map)),
        ]

        print(f"Navigating to cell {dest_cell} — press Ctrl+C to stop.\n")
        try:
            await navigation.go_to(dest_cell)
            print("\nDestination reached.")
        except (KeyboardInterrupt, asyncio.CancelledError):
            motion.stop()
        finally:
            for task in background_tasks:
                task.cancel()
            await asyncio.gather(*background_tasks, return_exceptions=True)
            path = navigation.map.save_map(notes=map_notes)
            print(f"Map saved to {path}")
            print("Shutdown complete.")

    dest_col = int(input("Destination cell X (col): ").strip())
    dest_row = int(input("Destination cell Y (row): ").strip())
    map_notes = input("Enter notes for the map (leave blank for none): ").strip()

    try:
        asyncio.run(main((dest_col, dest_row)))
    except KeyboardInterrupt:
        pass