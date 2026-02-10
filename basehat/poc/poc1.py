from buildhat import MotorPair, Motor
from basehat import LineFinder
import time

def main():
        # Initialize the motor pair on ports A and B
        motor_pair = MotorPair('A', 'B')
        line_finder_pin = 16

        linefinder = LineFinder(line_finder_pin)
        direction = 0
        while True:
                sensor_value = linefinder.value
                print(f"Line Finder Sensor Value: {sensor_value} Direction:{direction}")
                if sensor_value == 0:
                        if direction == 0:
                                motor_pair.start(-10, 0)  # Turn right
                                direction = 1
                        elif direction > 0:
                                if direction < 10:
                                        direction += 1
                                else:
                                        direction = -1
                                        motor_pair.start(-10, 0)  # Turn left
                        else:
                                if direction > -10:
                                        direction -= 1
                                else:
                                        direction = 1
                                        motor_pair.start(0, 10)  # Turn right
                else:
                        direction = 0
                        motor_pair.start(-10, 10)  # Move forward
                time.sleep(0.1)

if __name__ == "__main__":
        main()