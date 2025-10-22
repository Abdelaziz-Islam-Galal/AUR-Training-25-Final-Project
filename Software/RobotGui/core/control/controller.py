
import pygame
import math
from time import sleep
from RobotGui.core.control.robot_controller import RobotController
from threading import Thread

class Controller():
    def __init__(self, robot_controller_instance : RobotController):
        #initiating cmd receiver 
        self._robot_controller = robot_controller_instance

        #initiating the joystick
        pygame.init()
        pygame.joystick.init()
        #pygame.display.set_mode((1, 1))

        #initiating the joystick
        #pygame.init()
        #pygame.joystick.init()

        joystick = pygame.joystick.Joystick(0)
        joystick.init()

        #initiating the values used when an event happens
        self.eventValues=[0.0,0.0,0.0,0.0,0.0,0.0]
        self.magnitude = 0;self.angle = 0;self.xpressed = False;self.cpressed = False;self.spressed = False;self.tpressed = False;self.arm=0;self.gripper=0

        self._controller_thread = Thread(target=self.logic, daemon=True)
        self._controller_thread.start()

    def logic(self):
        while True:
            for event in pygame.event.get():
                if event.type == pygame.JOYAXISMOTION:
                    if abs(event.value) > 0.1: #deadzone of joystick
                        self.eventValues[event.axis] = event.value #storing values according to their axis
                    else :
                        self.eventValues[event.axis] = 0.0
                    
                    # calculating and reforming data to be sent
                    x = self.eventValues[0]
                    y = self.eventValues[1]

                    self.magnitude = math.sqrt((x*x)+(y*y)) #self.magnitude calculated

                    if x != 0:
                        self.angle = math.atan2(y , x) #from -pi/2 ------> pi/2
                        self.angle = math.degrees(self.angle)
                        if self.angle < 0:
                            self.angle = self.angle + 360
                        
                        self.angle = self.angle + 90
                        if self.angle < 0:
                            self.angle = self.angle + 360

                        self.angle = abs(self.angle) % 360 #self.angle calculated

                    if self.magnitude == 0 : #safety procedure
                        self.angle = 0
                    
                    print(f'{self.magnitude},{self.angle}')
            
                #on pushing event
                if event.type == pygame.JOYBUTTONDOWN:
                    if event.button == 0:   #X button -> arm down
                        self.arm = -1
                        #print("X is pressed")
                    elif event.button == 3: #triself.angle button -> arm up
                        self.arm = 1
                        #print("T is pressed")
                    else:
                        self.arm = 0

                    if event.button == 1: #circle button -> gripper open
                        self.gripper = 1
                        #print("C is pressed")
                    elif event.button == 2: #square button -> gripper close
                        self.gripper = 0
                        #print("S is pressed")


                    # on releasing event
                if event.type == pygame.JOYBUTTONUP:
                    if event.button == 0:
                        self.arm = 0
                        #print("X is released")

                    elif event.button == 1:
                        self.gripper = 0
                        #print("C is released")

                    elif event.button == 2:
                        self.gripper = 0
                        #print("S is released")

                    elif event.button == 3:
                        self.arm = 0
                        #print("T is released")


                self._robot_controller.command_list([self.magnitude, self.angle, self.arm, self.gripper])
            sleep(0.03)