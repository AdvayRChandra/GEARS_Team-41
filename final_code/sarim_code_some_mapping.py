import time
import math
import matplotlib.pyplot as plt  # Added for Python plotting
from basehat import UltrasonicSensor, IRSensor, IMUSensor
from buildhat import Motor

# --- CONFIGURATION ---
# Pins & Ports
PORT_LEFT_MOTOR  = 'A'
PORT_RIGHT_MOTOR = 'B'
PIN_ULTRA_FRONT  = 24
PIN_ULTRA_LEFT   = 22
PIN_ULTRA_RIGHT  = 26
PIN_IR_1         = 2

# Thresholds
DIST_THRESHOLD   = 5.0
IR_THRESHOLD     = 40
MAG_THRESHOLD    = 200

# Movement Settings (DPS)
DRIVE_SPEED_DPS  = 20
TURN_SPEED_DPS   = 10

class RoverMapper:
    def __init__(self):
        # Initialize Motors
        self.motor_left = Motor(PORT_LEFT_MOTOR)
        self.motor_right = Motor(PORT_RIGHT_MOTOR)
        
        # Initialize Sensors
        self.ultra_front = UltrasonicSensor(PIN_ULTRA_FRONT)
        self.ultra_left  = UltrasonicSensor(PIN_ULTRA_LEFT)
        self.ultra_right = UltrasonicSensor(PIN_ULTRA_RIGHT)
        self.ir          = IRSensor(PIN_IR_1, PIN_IR_1 + 1)
        self.imu         = IMUSensor()

        # Mapping State (for MATLAB graphing)
        self.x, self.y = 0.0, 0.0
        self.heading = 90.0
        
        # Integrated: Path lists for Python Plotting
        self.path_x = [0.0]
        self.path_y = [0.0]

        print("MATLAB Mapping Initialized:")
        print("figure; hold on; grid on; xlabel('X (cm)'); ylabel('Y (cm)');")

    def get_magnetic_magnitude(self):
        """Calculates total magnetic field strength."""
        mx, my, mz = self.imu.getMag()
        return math.sqrt(mx**2 + my**2 + mz**2)

    def turn_in_place(self, angle_deg):
        abs_angle = abs(angle_deg)
        if angle_deg > 0: # LEFT
            if abs_angle > 180: motor_degrees = 4 * abs_angle
            elif abs_angle >= 45: motor_degrees = 2 * abs_angle
            else: motor_degrees = 1.6 * abs_angle
            
            self.motor_left.run_for_degrees(motor_degrees, speed=TURN_SPEED_DPS, blocking=False)
            self.motor_right.run_for_degrees(motor_degrees, speed=TURN_SPEED_DPS, blocking=True)
              
        else: # RIGHT
            if abs_angle > 90: motor_degrees = 2.25 * abs_angle
            elif abs_angle >= 45: motor_degrees = 2 * abs_angle
            else: motor_degrees = 1.6 * abs_angle
            
            self.motor_left.run_for_degrees(-motor_degrees, speed=TURN_SPEED_DPS, blocking=False)
            self.motor_right.run_for_degrees(-motor_degrees, speed=TURN_SPEED_DPS, blocking=True)

        self.heading += angle_deg
        print(f"% Turning {angle_deg} degrees. New Heading: {self.heading}")

    def move_forward(self, duration=0.5):
        self.motor_left.start(-DRIVE_SPEED_DPS)
        self.motor_right.start(DRIVE_SPEED_DPS)
        
        time.sleep(duration)
        
        dist_moved = 15.0 * duration
        new_x = self.x + dist_moved * math.cos(math.radians(self.heading))
        new_y = self.y + dist_moved * math.sin(math.radians(self.heading))
        
        print(f"plot([{self.x:.2f} {new_x:.2f}], [{self.y:.2f} {new_y:.2f}], 'b-', 'LineWidth', 2);")
        
        # Update current coordinates
        self.x, self.y = new_x, new_y
        
        # Integrated: Save new coordinates to path lists
        self.path_x.append(self.x)
        self.path_y.append(self.y)

    def run(self):
        try:
            while True:
                raw_front = self.ultra_front.getDist
                raw_left  = self.ultra_left.getDist
                raw_right = self.ultra_right.getDist
                
                d_front = raw_front if raw_front is not None else 999.0
                d_left  = raw_left  if raw_left  is not None else 999.0
                d_right = raw_right if raw_right is not None else 999.0

                ir_l, ir_r = self.ir.value1, self.ir.value2
                ir_heat = max(ir_l if ir_l else 0, ir_r if ir_r else 0)
                
                mag = self.get_magnetic_magnitude()

                wall_front = d_front < 10.0
                heat_front = ir_heat > 40
                mag_front  = mag > 800

                if wall_front or heat_front or mag_front:
                    reason = ""
                    if wall_front: reason += f"Dist({d_front}) "
                    if heat_front: reason += f"Heat({ir_heat}) "
                    if mag_front:  reason += f"Mag({mag:.1f}) "
                    
                    print(f"% OBSTACLE DETECTED: {reason}")

                    if d_left < d_right:
                        print("% Turning Right (90)")
                        self.turn_in_place(-70)
                    else:
                        print("% Turning Left (90)")
                        self.turn_in_place(70)

                elif d_left < 5.0:
                    print(f"% Adjusting Right (10) - Left Wall: {d_left}")
                    self.turn_in_place(-10)

                elif d_right < 5.0:
                    print(f"% Adjusting Left (10) - Right Wall: {d_right}")
                    self.turn_in_place(10)

                else:
                    self.move_forward(0.5)

                time.sleep(0.1)

        except KeyboardInterrupt:
            print("\nStopping and generating Python plot...")
            # Integrated: Plot the gathered path data
            plt.figure(figsize=(10, 10))
            plt.plot(self.path_x, self.path_y, marker='o', linestyle='-', color='red', label='Robot Path')
            plt.title("Robot Maze Navigation Path")
            plt.xlabel("X Position (cm)")
            plt.ylabel("Y Position (cm)")
            plt.grid(True)
            plt.axis('equal')
            plt.legend()
            plt.savefig("map.png") # This will hold the script until the window is closed
            
        finally:
            self.motor_left.stop()
            self.motor_right.stop()

if __name__ == "__main__":
    rover = RoverMapper()
    rover.run()