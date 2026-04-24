from buildhat import Motor
from .state import State


class MotionController:
    """Class for controlling the robot's motion."""

    def __init__(self, state: State, **kwargs):
        self.state = state
        self.motor_left = Motor('A')   # Left motor  — port A
        self.motor_right = Motor('B')  # Right motor — port B

    def _sync_state(self):
        """Update state.motor_* with current hardware readings."""
        self.state.motor_left.position = self.motor_left.get_position()
        self.state.motor_right.position = self.motor_right.get_position()
        self.state.motor_left.is_moving = self.motor_left.get_speed() != 0
        self.state.motor_right.is_moving = self.motor_right.get_speed() != 0

    async def update_state(self):
        """Sync motor state once. Call this each control loop tick."""
        self._sync_state()

    def _set_moving(self, moving: bool):
        """Directly set is_moving on both motor state entries."""
        self.state.motor_left.is_moving = moving
        self.state.motor_right.is_moving = moving

    def forward(self, speed: int = 50):
        """Drive both motors forward."""
        self.motor_left.start(speed)
        self.motor_right.start(speed)
        self._set_moving(True)
        self._sync_state()

    def backward(self, speed: int = 50):
        """Drive both motors backward."""
        self.motor_left.start(-speed)
        self.motor_right.start(-speed)
        self._set_moving(True)
        self._sync_state()

    def turn_left(self, speed: int = 50):
        """Pivot left: left motor backward, right motor forward."""
        self.motor_left.start(-speed)
        self.motor_right.start(speed)
        self._set_moving(True)
        self._sync_state()

    def turn_right(self, speed: int = 50):
        """Pivot right: left motor forward, right motor backward."""
        self.motor_left.start(speed)
        self.motor_right.start(-speed)
        self._set_moving(True)
        self._sync_state()

    def stop(self):
        """Stop both motors."""
        self.motor_left.stop()
        self.motor_right.stop()
        self._set_moving(False)
        self._sync_state()

