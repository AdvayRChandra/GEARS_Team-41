from buildhat import Motor, ColorSensor

class Cargo:
    def __init__(self, **kwargs):
        self.color_sensor = ColorSensor(kwargs.get("color_port", 'D'))
        self.cargo_motor = Motor(kwargs.get("cargo_motor_port", 'C'))
        self.speed = kwargs.get("cargo_motor_speed", 0.7)
    
    def activate_light(self):
        """Activates the color sensor's light with the specified color."""
        self.color_sensor.on()
    
    def deactivate_light(self):
        """Deactivates the color sensor's light."""
        self.color_sensor.off()
    
    def deploy_cargo(self, speed: int = 50):
        """Deploys the cargo by running the motor at the specified speed."""
        self.cargo_motor.run_for_degrees(120, speed)
        self.deactivate_light()
    
    def retract_cargo(self, speed: int = 50):
        self.cargo_motor.run_for_degrees(-120, speed)
        self.activate_light()