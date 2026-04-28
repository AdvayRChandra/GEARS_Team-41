import time
import math
from basehat import UltrasonicSensor, IRSensor, IMUSensor
from buildhat import Motor

# --- CONFIGURATION ---
# Pins & Ports
PORT_LEFT_MOTOR  = 'A'
PORT_RIGHT_MOTOR = 'B'
PIN_ULTRA_FRONT  = 24
PIN_ULTRA_LEFT   = 22
PIN_ULTRA_RIGHT  = 26 # Assuming D22 for the third sensor
PIN_IR_1         = 2  # Analog A2 (uses 2 and 3)

# Thresholds
DIST_THRESHOLD   = 5.0  # cm
IR_THRESHOLD     = 40    # Heat units
MAG_THRESHOLD    = 200   # uT

# Movement Settings (DPS)
DRIVE_SPEED_DPS  = 20   # Speed for forward movement
TURN_SPEED_DPS   = 10   # Speed for turning

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
        self.heading = 90.0  # Start facing "North" (90 deg)
        print("MATLAB Mapping Initialized:")
        print("figure; hold on; grid on; xlabel('X (cm)'); ylabel('Y (cm)');")

    def get_magnetic_magnitude(self):
        """Calculates total magnetic field strength."""
        mx, my, mz = self.imu.getMag()
        return math.sqrt(mx**2 + my**2 + mz**2)

    def turn_in_place(self, angle_deg):
        """
        Adapts logic from robot_angle_code.py using DPS and degree calculations.
        +angle = left, -angle = right.
        """
        abs_angle = abs(angle_deg)
        # Logic from robot_angle_code.py for motor rotation calculation
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

        # Update internal heading for mapping
        self.heading += angle_deg
        print(f"% Turning {angle_deg} degrees. New Heading: {self.heading}")

    def move_forward(self, duration=0.5):
        """Moves forward at a set DPS and updates (x,y) coordinates."""
        # Start motors at constant DPS
        self.motor_left.start(-DRIVE_SPEED_DPS) # One motor usually inverted
        self.motor_right.start(DRIVE_SPEED_DPS)
        
        time.sleep(duration)
        
        #self.motor_left.stop()
        #self.motor_right.stop()

        # Calculate distance traveled (Estimated based on DPS and wheel size)
        # Dist = (DPS / 360) * WheelCircumference * duration. 
        # Assuming ~15cm traveled per second for this example:
        dist_moved = 15.0 * duration 
        
        new_x = self.x + dist_moved * math.cos(math.radians(self.heading))
        new_y = self.y + dist_moved * math.sin(math.radians(self.heading))
        
        # MATLAB output
        print(f"plot([{self.x:.2f} {new_x:.2f}], [{self.y:.2f} {new_y:.2f}], 'b-', 'LineWidth', 2);")
        self.x, self.y = new_x, new_y

    def run(self):
        try:
            while True:
                # --- 1. READ SENSORS WITH NONETYPE PROTECTION ---
                raw_front = self.ultra_front.getDist #
                raw_left  = self.ultra_left.getDist
                raw_right = self.ultra_right.getDist
                
                # Treat None as 999cm (no wall)
                d_front = raw_front if raw_front is not None else 999.0
                d_left  = raw_left  if raw_left  is not None else 999.0
                d_right = raw_right if raw_right is not None else 999.0

                ir_l, ir_r = self.ir.value1, self.ir.value2 #
                ir_heat = max(ir_l if ir_l else 0, ir_r if ir_r else 0)
                
                mag = self.get_magnetic_magnitude() #

                # --- 2. DETERMINE TRIGGER REASONS ---
                wall_front = d_front < 10.0
                heat_front = ir_heat > 40
                mag_front  = mag > 800

                # --- 3. DECISION LOGIC ---
                if wall_front or heat_front or mag_front:
                    # Determine specific reason for turning
                    reason = ""
                    if wall_front: reason += f"Dist({d_front}) "
                    if heat_front: reason += f"Heat({ir_heat}) "
                    if mag_front:  reason += f"Mag({mag:.1f}) "
                    
                    print(f"% OBSTACLE DETECTED: {reason}")

                    # Logic for front obstacle
                    if d_left < d_right:
                        print("% Turning Right (90)")
                        self.turn_in_place(-70)
                    else:
                        print("% Turning Left (90)")
                        self.turn_in_place(70)

                elif d_left < 5.0:
                    print(f"% Adjusting Right (20) - Left Wall: {d_left}")
                    self.turn_in_place(-10)

                elif d_right < 5.0:
                    print(f"% Adjusting Left (20) - Right Wall: {d_right}")
                    self.turn_in_place(10)

                else:
                    # Only move forward if NO sensors are triggered
                    self.move_forward(0.5)

                time.sleep(0.1)

        except KeyboardInterrupt:
            print("Stopping...")
        finally:
            self.motor_left.stop()
            self.motor_right.stop()

if __name__ == "__main__":
    rover = RoverMapper()
    rover.run()