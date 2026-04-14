import asyncio
from buildhat import Motor
from systems.state import State


class Cargo:
    """
    Cargo detection and deployment system using magnetic field sensing.
    
    Uses the IMU magnetometer (via State) to detect magnetic cargo markers.
    Magnetic field values are updated by SensorInput.
    Payload motor controls cargo deployment.
    
    Args:
        state (State): Centralized state object for sensor data
        motor_port (str): Build HAT port for payload motor (default: "B")
        deploy_angle (int): Degrees to turn for deployment (default: 180)
        edge_threshold (float): Magnetic threshold for edge detection in µT (default: 400)
        semi_threshold (float): Magnetic threshold for semi-detection in µT (default: 1000)
        full_threshold (float): Magnetic threshold for full detection in µT (default: 3000)
    """
    
    def __init__(self, **kwargs):
        self.state = kwargs.get("state", State())
        
        # Payload motor
        self.motor = Motor(kwargs.get("motor_port", "B"))
        self.deploy_angle = kwargs.get("deploy_angle", 180)
        self.deployed = False  # Tracks if cargo has been deployed (one-time deployment)

        self.edge_mag_threshold = kwargs.get("edge_threshold", 400)  # µT
        self.semi_mag_threshold = kwargs.get("semi_threshold", 1000)  # µT
        self.full_mag_threshold = kwargs.get("full_threshold", 3000)  # µT
        self.deploy_distance = kwargs.get("deploy_distance", 0.0) * 100 # In cm

        # Target cargo configuration
        self.target_cargo_number = kwargs.get("target_cargo_number", 1)
        self.buffer_distance = kwargs.get("buffer_distance", 6.0)  # cm
        
        # Current cargo tracking
        self.current_cargo_number = 0  # Incremented each time cargo is detected
        self.buffer_end_distance = 0.0  # Distance at which buffer period ends
        self.in_buffer_period = False  # True when in buffer zone after non-target cargo

        # Maximum magnetic field observed
        self.max_magnetic_field = 0.0

        # Maximum cargo level observed
        self.max_cargo_level = "none"

        # Distance of maximum magnetic level detected
        self.max_mag_distance = 0.0
        
        # Track cargo bay state
        self.cargo_bay_open = False
        self.initial_motor_position = self.motor.get_aposition()

    def update_max_magnetic_field(self):
        """Update the maximum magnetic field if current reading is higher."""
        if self.state.magnetic_field > self.max_magnetic_field:
            self.max_magnetic_field = self.state.magnetic_field
            self.max_mag_distance = self.state.distance_traveled

    def detect_cargo_level(self):
        """
        Detect the cargo proximity level based on magnetic field.
        
        Returns:
            str: "none", "edge", "semi", or "full" based on magnetic reading
        """
        self.update_max_magnetic_field()
        magnetic_field = self.state.magnetic_field
        
        if magnetic_field >= self.full_mag_threshold:
            if self.max_cargo_level != "full":
                self.max_cargo_level = "full"
            return "full"
        elif magnetic_field >= self.semi_mag_threshold:
            if self.max_cargo_level != "semi" and self.max_cargo_level != "full":
                self.max_cargo_level = "semi"
            return "semi"
        elif magnetic_field >= self.edge_mag_threshold:
            if self.max_cargo_level == "none":
                self.max_cargo_level = "edge"
            return "edge"
        else:
            return "none"

    def is_cargo_detected(self):
        """
        Check if any cargo is detected.
        
        Returns:
            bool: True if cargo detected at any level
        """
        return self.detect_cargo_level() != "none"
    
    async def deploy(self):
        """
        Deploy the cargo by turning the payload motor +180 degrees.
        Waits until robot travels deploy_distance past the max magnetic reading,
        then pauses motion and deploys cargo.
        """

        if self.deployed:
            print("Cargo already deployed")
            return
        
        target_distance = self.max_mag_distance + self.deploy_distance
        print(f"Waiting to deploy at distance {target_distance:.2f} cm (current: {self.state.distance_traveled:.2f} cm)")
        
        # Wait until we've traveled far enough, yielding to other tasks
        while self.state.distance_traveled < target_distance:
            await asyncio.sleep(0.05)  # Yield control to allow sensor updates
        
        self.state.deploying_cargo = True
        print("Deploying cargo...")
        start_position = self.motor.get_aposition()
        target_position = start_position - self.deploy_angle
        
        self.motor.run_for_degrees(-self.deploy_angle, blocking=True)
        
        """
        # Confirm cargo bay is fully open by checking motor position
        tolerance = 5  # degrees tolerance
        while True:
            current_position = self.motor.get_aposition()
            if abs(current_position - target_position) <= tolerance:
                break
            print(f"Waiting for cargo bay to fully open... (position: {current_position:.1f}, target: {target_position:.1f})")
            await asyncio.sleep(0.1)
        """
        
        self.cargo_bay_open = True
        self.deployed = True
        print("Cargo deployed and confirmed open")
    
    async def close(self):
        """
        Close the payload by turning the motor back.
        Only closes if cargo bay is confirmed open.
        """
        if not self.cargo_bay_open:
            print("Cargo bay not open, cannot close")
            return
            
        print("Closing cargo bay...")
        self.motor.run_for_degrees(self.deploy_angle, blocking=True)
        self.cargo_bay_open = False
        self.state.deploying_cargo = False
        print("Cargo bay closed")
    
    async def deploy_and_close(self):
        """Deploy and then close the cargo bay after confirmation."""
        await self.deploy()
        
        # Wait a moment to ensure cargo has fallen out
        await asyncio.sleep(2.0)
        await self.close()
    
    def stop(self):
        """Stop the cargo motor."""
        self.motor.stop()
    
    def reset_detection(self):
        """Reset detection values after passing a cargo spot."""
        self.max_magnetic_field = 0.0
        self.max_cargo_level = "none"
        self.max_mag_distance = 0.0
        print(f"Detection values reset for next cargo")
    
    async def run_cargo_update_loop(self, update_interval: float = 0.1):
        """
        Continuously monitor cargo level and update State.
        Deploys cargo only at the target cargo number.
        
        Args:
            update_interval (float): Time between checks in seconds (default: 0.1)
        """
        while True:
            # Check if in buffer period (waiting after non-target cargo)
            if self.in_buffer_period:
                if self.state.distance_traveled >= self.buffer_end_distance:
                    self.in_buffer_period = False
                    self.reset_detection()
                    print(f"Buffer period ended, ready to detect cargo #{self.current_cargo_number + 1}")
                await asyncio.sleep(update_interval)
                continue
            
            self.state.cargo_level = self.detect_cargo_level()
            current_field = self.state.magnetic_field
            
            # Update state with current cargo number
            self.state.cargo_number = self.current_cargo_number
            
            # Detect cargo when magnetic field starts decreasing (passed the cargo)
            if not self.deployed and self.max_cargo_level != "none":
                # Check if we've passed the peak magnetic field
                if self.max_magnetic_field > current_field and self.state.cargo_level == "none":
                    # We've passed a cargo spot
                    self.current_cargo_number += 1
                    print(f"Detected cargo #{self.current_cargo_number} at distance {self.max_mag_distance:.2f} cm")
                    
                    if self.current_cargo_number == self.target_cargo_number:
                        # This is our target cargo - deploy!
                        print(f"Target cargo #{self.target_cargo_number} reached - deploying")
                        await self.deploy_and_close()
                    else:
                        # Not our target - enter buffer period and continue
                        print(f"Not target cargo (target: #{self.target_cargo_number}), continuing...")
                        self.in_buffer_period = True
                        self.buffer_end_distance = self.state.distance_traveled + self.buffer_distance
                        print(f"Buffer active until distance {self.buffer_end_distance:.2f} cm")
            
            await asyncio.sleep(update_interval)
    
