from buildhat.motors import Motor


class Motion:
    def __init__(self, **kwargs):
        self._motor_left = Motor(kwargs.get("motor_left"))
        self._motor_right = Motor(kwargs.get("motor_right"))
        self._speed = kwargs.get("speed", 50)
    
    def forward(self):
        self._motor_left.start(self._speed)
        self._motor_right.start(self._speed)
    
