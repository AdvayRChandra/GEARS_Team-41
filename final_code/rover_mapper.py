# none logic, light, cargo, mapping w hazards, turns for 180

import time
import math
import csv  # Added for hazard file output
import matplotlib.pyplot as plt  # Added for Python plotting
from basehat import UltrasonicSensor, IRSensor, IMUSensor
from buildhat import Motor
from buildhat import ColorSensor

# --- CONFIGURATION ---
PORT_LEFT_MOTOR  = 'A'
PORT_RIGHT_MOTOR = 'B'
PORT_BACK_MOTOR = 'C'
PORT_COLOR = 'D'
PIN_ULTRA_FRONT  = 22
PIN_ULTRA_LEFT   = 24
PIN_ULTRA_RIGHT  = 26
PIN_IR_1         = 2

# Thresholds
DIST_THRESHOLD   = 5.0
IR_THRESHOLD     = 100
MAG_THRESHOLD    = 500

# Movement Settings (DPS)
DRIVE_SPEED_DPS  = 20
TURN_SPEED_DPS   = 15

class RoverMapper:
    def __init__(self):
        # Initialize Motors
        self.motor_left = Motor(PORT_LEFT_MOTOR)
        self.motor_right = Motor(PORT_RIGHT_MOTOR)
        self.motor_back = Motor(PORT_BACK_MOTOR)
        self.color = ColorSensor(PORT_COLOR)
        
        # Initialize Sensors
        self.ultra_front = UltrasonicSensor(PIN_ULTRA_FRONT)
        self.ultra_left  = UltrasonicSensor(PIN_ULTRA_LEFT)
        self.ultra_right = UltrasonicSensor(PIN_ULTRA_RIGHT)
        self.ir          = IRSensor(PIN_IR_1, PIN_IR_1 + 1)
        self.imu         = IMUSensor()

        # Mapping State
        self.x, self.y = 0.0, 0.0
        self.heading = 90.0
        
        self.path_x = [0.0]
        self.path_y = [0.0]
        
        # --- NEW: HAZARD TRACKING ---
        self.hazards_list = []
        self.heat_points_x = []
        self.heat_points_y = []
        self.mag_points_x = []
        self.mag_points_y = []
        self.exit_point = None

        # --- GYRO INTEGRATION SETUP ---
        self.last_time = time.time()
        print("Calibrating Gyro... Please keep the robot still.")
        # Taking 50 samples to find the 'idle' noise (the 1,0,1 you mentioned)
        bias_samples = []
        for _ in range(50):
            bias_samples.append(self.imu.getGyro())
            time.sleep(0.01)
        
        # We calculate the average noise for each axis
        self.bias_x = sum(s[0] for s in bias_samples) / 50
        self.bias_y = sum(s[1] for s in bias_samples) / 50
        self.bias_z = sum(s[2] for s in bias_samples) / 50
        print(f"Calibration Complete. Z-Bias: {self.bias_z:.4f}")

        print("MATLAB Mapping Initialized:")
        print("figure; hold on; grid on; xlabel('X (cm)'); ylabel('Y (cm)');")

    def update_gyro_heading(self):
        """Integrates the Gyro Z-axis to update the robot heading."""
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # Get raw gyro readings
        _, _, gz = self.imu.getGyro()
        
        # Subtract bias and integrate: angle = speed * time
        # Note: If your robot turns 'left' and heading goes down, 
        # you may need to flip the sign here (e.g., -=)
        actual_gz = gz - self.bias_z
        
        # Only update if rotation is above a small noise threshold
        if abs(actual_gz) > 0.1: 
            self.heading += actual_gz * dt

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
            self.motor_right.run_for_degrees(motor_degrees, speed=TURN_SPEED_DPS, blocking=False)
              
        else: # RIGHT
            if abs_angle > 90: motor_degrees = 2.25 * abs_angle
            elif abs_angle >= 45: motor_degrees = 2 * abs_angle
            else: motor_degrees = 1.6 * abs_angle
            
            self.motor_left.run_for_degrees(-motor_degrees, speed=TURN_SPEED_DPS, blocking=False)
            self.motor_right.run_for_degrees(-motor_degrees, speed=TURN_SPEED_DPS, blocking=False)

        # Replacing .is_running() with a speed-check loop
        # We also add a small timeout to prevent infinite loops if a motor stalls
        start_turn_time = time.time()
        expected_duration = (motor_degrees / TURN_SPEED_DPS) + 1.0 
        
        self.last_time = time.time()
        while (time.time() - start_turn_time) < expected_duration:
            self.update_gyro_heading()
            # If both motors have stopped moving, we can exit the loop early
            if abs(self.motor_left.get_speed()) < 1 and abs(self.motor_right.get_speed()) < 1:
                if (time.time() - start_turn_time) > 0.2: # Small buffer to let them start
                    break
            time.sleep(0.01)

        print(f"% Turn Finished. Gyro Heading: {self.heading:.2f}")

    def move_forward(self, duration=0.5):
        self.motor_left.start(-DRIVE_SPEED_DPS)
        self.motor_right.start(DRIVE_SPEED_DPS)
        
        # Track heading while moving straight (to catch drifting)
        start_move = time.time()
        self.last_time = start_move
        while (time.time() - start_move) < duration:
            self.update_gyro_heading()
            time.sleep(0.01)
            
#        self.motor_left.stop()
 #       self.motor_right.stop()
        
        dist_moved = 15.0 * duration
        new_x = self.x + dist_moved * math.cos(math.radians(self.heading))
        new_y = self.y + dist_moved * math.sin(math.radians(self.heading))
        
        print(f"plot([{self.x:.2f} {new_x:.2f}], [{self.y:.2f} {new_y:.2f}], 'b-', 'LineWidth', 2);")
        
        self.x, self.y = new_x, new_y
        self.path_x.append(self.x)
        self.path_y.append(self.y)

    def write_hazard_file(self):
        """Generates the team41_hazards.csv file."""
        with open('team41_hazards.csv', 'w', newline='') as f:
            f.write("Team: 41\nMap: 2\nNotes: Automated hazard summary\n")
            writer = csv.writer(f)
            writer.writerow(["Hazard Type", "Parameter of Interest", "Parameter Value", "Hazard X Coordinate (cm)", "Hazard Y Coordinate (cm)"])
            writer.writerows(self.hazards_list)
        print("Hazard file 'team41_hazards.csv' generated.")

    def run(self):
        try:
            while True:
                # Assuming these are properties in your specific library
                raw_front = self.ultra_front.getDist
                raw_left  = self.ultra_left.getDist
                raw_right = self.ultra_right.getDist
                self.color.on()
                self.motor_back.run_for_degrees(-20)
                
                
                d_front = raw_front if raw_front is not None else 999.0
                d_left  = raw_left  if raw_left  is not None else 999.0
                d_right = raw_right if raw_right is not None else 999.0

                ir_l, ir_r = self.ir.value1, self.ir.value2
                ir_heat = max(ir_l if ir_l else 0, ir_r if ir_r else 0)
                
                mag = self.get_magnetic_magnitude()

                wall_front = d_front < 17.5
                heat_front = ir_heat > 100
                mag_front  = mag > 400

                if wall_front or heat_front or mag_front:
                    reason = ""
                    if wall_front: reason += f"Dist({d_front}) "
                    if heat_front: 
                        reason += f"Heat({ir_heat}) "
                        # LOG HEAT HAZARD
                        self.hazards_list.append(["High Temperature Heat Source", "Radiated Power (W)", round(ir_heat, 2), round(self.x, 2), round(self.y, 2)])
                        rad = math.radians(self.heading)
                        self.heat_points_x.append(self.x + 5 * math.cos(rad))
                        self.heat_points_y.append(self.y + 5 * math.sin(rad))
                    if mag_front:  
                        reason += f"Mag({mag:.1f}) "
                        # LOG MAG HAZARD
                        self.hazards_list.append(["Electrical/Magnetic Activity Source", "Field Strength (uT)", round(mag, 2), round(self.x, 2), round(self.y, 2)])
                        rad = math.radians(self.heading)
                        self.mag_points_x.append(self.x + 5 * math.cos(rad))
                        self.mag_points_y.append(self.y + 5 * math.sin(rad))
                    
                    print(f"% OBSTACLE DETECTED: {reason}")

                    if d_left < d_right:
                        print("% Turning Right (90)")
                        self.turn_in_place(-75)
                    else:
                        print("% Turning Left (90)")
                        self.turn_in_place(75)

                elif d_left < 8.0:
                    print(f"% Adjusting Right (10) - Left Wall: {d_left}")
                    self.turn_in_place(-15)
                    #self.move_forward(1)

                elif d_right < 10.0:
                    print(f"% Adjusting Left (10) - Right Wall: {d_right}")
                    self.turn_in_place(70)
                    #self.move_forward(1)
                elif d_left > 50 and d_right > 50 and d_front > 50:
                    self.motor_back.run_for_degrees(160)
                    self.color.off()
                    self.exit_point = (self.x, self.y)# Mark final point
                    time.sleep(1)
                    self.move_forward(1)
                    self.motor_right.stop()
                    self.motor_left.stop()
                    time.sleep(5)
                    break # Break to finish and generate plot/file
                    
                else:
                    self.move_forward(0.5)

                self.update_gyro_heading()
                time.sleep(0.1)

        except KeyboardInterrupt:
            print("\nStopping...")
            
        finally:
            # Generate the hazard CSV
            self.write_hazard_file()

            # Enhanced Python plot
            print("Generating map2.png...")
            plt.figure(figsize=(10, 10))
            # 1. Path
            plt.plot(self.path_x, self.path_y, marker='o', markersize=3, linestyle='-', color='blue', label='Robot Path', alpha=0.6)
            
            # 2. Heat Sources (Red)
            if self.heat_points_x:
                plt.scatter(self.heat_points_x, self.heat_points_y, color='red', s=100, label='Heat Source', zorder=5)
            
            # 3. Magnetic Sources (Black)
            if self.mag_points_x:
                plt.scatter(self.mag_points_x, self.mag_points_y, color='black', s=100, label='Magnetic Source', zorder=5)
            
            # 4. Final Point (Cross)
            if self.exit_point:
                plt.scatter(self.exit_point[0], self.exit_point[1], color='green', marker='x', s=200, linewidths=3, label='Exit Point', zorder=6)
            elif self.path_x: # If interrupted before formal exit
                plt.scatter(self.path_x[-1], self.path_y[-1], color='darkred', marker='x', s=200, linewidths=3, label='Interrupt Point', zorder=6)

            plt.title("GEARS Navigation Map - Team 41")
            plt.xlabel("X Position (cm)")
            plt.ylabel("Y Position (cm)")
            plt.grid(True)
            plt.axis('equal')
            plt.legend()
            plt.savefig("map2.png") 
            
            self.color.off()
            self.motor_left.stop()
            self.motor_right.stop()
            self.motor_back.stop()

if __name__ == "__main__":
    rover = RoverMapper()
    rover.run()