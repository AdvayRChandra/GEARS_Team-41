import time
import os
import math
from basehat import (
    UltrasonicSensor, Button, HallSensor, 
    IMUSensor, IRSensor, LightSensor, LineFinder
)
from buildhat import ColorSensor

def main():
    # ---- INITIALIZATION (Based on your sample code logic) ----
    
    # Ultrasonic Sensors
    # Assuming Front=5, Left=18, Right=22 per previous configuration
    ultra_front = UltrasonicSensor(24)
    ultra_left  = UltrasonicSensor(22)
    ultra_right = UltrasonicSensor(26)

    # BuildHAT Color Sensor
    # Port 'C' used here to avoid conflict with motors on A and B
 #   color_sen = ColorSensor('D')

    # IR Sensor
    # Analog Port A2 (Pins 2 and 3)
    ir_sen = IRSensor(2, 3)

    # IMU Sensor (Internal I2C)
    imu = IMUSensor()

    # Digital Sensors (Adjust pins as needed to avoid Pin 5 overlap)
  #  btn = Button(24)          #
   # hall = HallSensor(26)     #
    #line = LineFinder(16)     #
   # light = LightSensor(4)    #

    print("--- Starting Full Sensor Diagnostic Report ---")
    print("Press Ctrl+C to stop.")

    try:
        while True:
            # Clear terminal for a clean dashboard (optional)
            os.system('cls' if os.name == 'nt' else 'clear')
            
           # print("=== DISTANCE & NAVIGATION ===")
            # Ultrasonic getDist is an attribute, no parens
            print(f"Ultra Front: {ultra_front.getDist} cm")
            print(f"Ultra Left:  {ultra_left.getDist} cm")
            print(f"Ultra Right: {ultra_right.getDist} cm")

           # print("\n=== ENVIRONMENT & HEAT ===")
            # IR values are attributes
            print(f"IR Values: L:{ir_sen.value1} | R:{ir_sen.value2}")
            # Light value is an attribute
            #print(f"Light Level: {light.light}")
            # Color sensors use methods
            #print(f"Color: {color_sen.get_color()} | RGBI: {color_sen.get_color_rgbi()}")

         #   print("\n=== IMU (MOTION & MAGNETS) ===")
            # IMU uses methods for Accel, Gyro, and Mag
            ax, ay, az = imu.getAccel()
            gx, gy, gz = imu.getGyro()
            mx, my, mz = imu.getMag()
            print(f"Accel: X:{ax:7.2f} Y:{ay:7.2f} Z:{az:7.2f}")
            print(f"Gyro:  X:{gx:7.2f} Y:{gy:7.2f} Z:{gz:7.2f}")
            print(f"Mag:   X:{mx:7.2f} Y:{my:7.2f} Z:{mz:7.2f}")
            print(f"Magnet:   {math.sqrt(mx**2+my**2+mz**2)}")

          #  print("\n=== DIGITAL STATES ===")
            # Button, Hall, and LineFinder use .value
 #           print(f"Button Pressed: {btn.value}")
#            print(f"Magnet Detected (Hall): {hall.value}")
          #  print(f"Line Detected: {line.value}")

            time.sleep(0.5)

    except IOError:
        print("\nError: Sensor communication lost.")
    except KeyboardInterrupt:
        print("\nDiagnostic terminated by user.")

if __name__ == "__main__":
    main()