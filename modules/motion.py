import asyncio
import serial
from buildhat import Motor
from .state import State
from .config import RobotConfig


def _flush_serial(device: str = "/dev/serial0") -> None:
    """Discard accumulated HAT data from the OS serial buffer.

    The BuildHAT continuously streams sensor/motor data while running.
    If this data accumulates before BuildHAT.__init__ reads the serial port,
    the unrecognized lines overflow the retry counter and raise BuildHATError.
    """
    try:
        with serial.Serial(device, 115200, timeout=0.05) as s:
            s.reset_input_buffer()
    except serial.SerialException:
        pass


class MotionController:
    """Class for controlling the robot's motion."""

    def __init__(self, state: State, config: RobotConfig):
        _flush_serial()
        self.state = state
        self.speed = config.motor_speed
        self.motor_left = Motor(config.motors.port_left)
        self.motor_right = Motor(config.motors.port_right)

    def _sync_state(self):
        """Update state.motor_* with current hardware readings."""
        self.state.motor_left.position = self.motor_left.get_position()
        self.state.motor_right.position = self.motor_right.get_position()
        self.state.motor_left.is_moving = self.motor_left.get_speed() != 0
        self.state.motor_right.is_moving = self.motor_right.get_speed() != 0

    async def update_state(self):
        """Sync motor state once. Call this each control loop tick."""
        self._sync_state()

    def _drive(self, left: int, right: int):
        """Start both motors at the given signed speeds, then sync state."""
        self.motor_left.start(left)
        self.motor_right.start(right)
        self.state.motor_left.is_moving = True
        self.state.motor_right.is_moving = True
        self._sync_state()

    def forward(self, speed: int = None):
        """Drive both motors forward."""
        s = speed if speed is not None else self.speed
        self._drive(s, -s)

    def backward(self, speed: int = None):
        """Drive both motors backward."""
        s = speed if speed is not None else self.speed
        self._drive(-s, s)

    def turn_right(self, speed: int = None):
        """Pivot left: left motor backward, right motor forward."""
        s = speed if speed is not None else self.speed
        self._drive(-s, -s)

    def turn_left(self, speed: int = None):
        """Pivot right: left motor forward, right motor backward."""
        s = speed if speed is not None else self.speed
        self._drive(s, s)

    def stop(self):
        """Stop both motors."""
        self.motor_left.stop()
        self.motor_right.stop()
        self.state.motor_left.is_moving = False
        self.state.motor_right.is_moving = False
        self._sync_state()

    async def setup(self):
        """Sync motor state once before the update loop starts."""
        self._sync_state()

    async def run_motor_update(self, **kwargs):
        """
        Continuously sync motor positions and motion status into state.

        Location reads state.motor_left.position and state.motor_right.position
        every tick. Without this loop those values would only update on drive
        commands, making odometry stale whenever the robot is coasting or idle.

        Args:
            update_interval (float): Update interval in seconds (default: 0.05)
        """
        update_interval = kwargs.get("update_interval", 0.05)
        while True:
            self._sync_state()
            await asyncio.sleep(update_interval)

