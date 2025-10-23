import pygame
import math
from RobotGui.core.cv.cv import Camera
from RobotGui.core.control.robot_controller import RobotController
from threading import Thread
from time import sleep

class Controller():
    def __init__(self, robot_controller_instance: RobotController):
        # Store robot controller instance
        self._robot_controller = robot_controller_instance
        self.CamInstance = Camera

        # Initialize control values
        self.eventValues = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.magnitude = 0
        self.angle = 0
        self.arm = 0
        self.gripper = 0
        self.isAllZeros = False
        
        # Add running flag
        self._running = False

        # Initialize pygame and joystick in main thread
        pygame.init()
        pygame.joystick.init()
        
        # Check if joystick is available
        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            print(f"Joystick detected: {self.joystick.get_name()}")
        else:
            self.joystick = None
            print("No joystick detected!")

        # Start controller thread
        self._controller_thread = Thread(target=self.logic, daemon=True)
        self._running = True
        self._controller_thread.start()

    def logic(self):
        """Run in separate thread - handles joystick logic"""
        while self._running:
            # Process all pending events
            for event in pygame.event.get():
                self._handle_event(event)
            
            # Send commands to robot
            if self.magnitude == 0 and self.angle == 0 and self.arm == 0 and self.gripper == 0:
                if self.isAllZeros == False:
                    self._robot_controller.command_list([self.magnitude, self.angle, self.arm, self.gripper])
                    self.isAllZeros = True
            else:
                self._robot_controller.command_list([self.magnitude, self.angle, self.arm, self.gripper])
                self.isAllZeros = False
            
            # Small delay to prevent excessive CPU usage
            sleep(0.01)  # Use time.sleep instead of pygame.delay in thread

    def _handle_event(self, event):
        """Handle individual pygame events"""
        if event.type == pygame.JOYAXISMOTION:
            if abs(event.value) > 0.1:  # deadzone of joystick
                self.eventValues[event.axis] = event.value
            else:
                self.eventValues[event.axis] = 0.0
            
            # Calculate magnitude and angle
            x = self.eventValues[0]
            y = self.eventValues[1]

            self.magnitude = min(math.sqrt((x*x) + (y*y)), 1.0)  # Clamp to 1.0

            if x != 0 or y != 0:
                self.angle = math.degrees(math.atan2(y, x))
                # Convert to 0-360 range with 0° at top
                self.angle = (450 - self.angle) % 360
            else:
                self.angle = 0

            #print(f'{self.magnitude:.2f}, {self.angle:.1f}')

        elif event.type == pygame.JOYBUTTONDOWN:
            if event.button == 0:   # X button -> arm down
                self.arm = -1
            elif event.button == 3: # triangle button -> arm up
                self.arm = 1
                
            if event.button == 1:   # circle button -> gripper open
                self.gripper = 1
            elif event.button == 2: # square button -> gripper close
                self.gripper = -1

            if event.button == 5:   # R1 button -> start QR scanning
                if hasattr(self.CamInstance, '_qr_thread'):
                    self.CamInstance._qr_thread.start()

        elif event.type == pygame.JOYBUTTONUP:
            if event.button in [0, 3]:  # X or triangle released
                self.arm = 0
            elif event.button in [1, 2]:  # circle or square released
                self.gripper = 0

    def stop(self):
        """Stop the controller thread and clean up"""
        self._running = False
        if self._controller_thread.is_alive():
            self._controller_thread.join(timeout=1.0)
            
        # Send final stop command
        self._robot_controller.command_list([0, 0, 0, 0])
        
        # Clean up pygame
        pygame.quit()

    def __del__(self):
        """Destructor to ensure cleanup"""
        self.stop()

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.stop()