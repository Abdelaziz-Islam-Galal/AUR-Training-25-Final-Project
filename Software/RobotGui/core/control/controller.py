import pygame
import math
from RobotGui.core.control.robot_controller import RobotController

#initiating cmd receiver 
robotcontroller = RobotController()

#initiating the joystick
pygame.init()
pygame.joystick.init()

joystick = pygame.joystick.Joystick(0)
joystick.init()

#initiating the values used when an event happens
eventValues=[0.0,0.0,0.0,0.0,0.0,0.0]
magnitude = 0;angle = 0;xpressed = False;cpressed = False;spressed = False;tpressed = False

while True:
    for event in pygame.event.get():
        if event.type == pygame.JOYAXISMOTION:
            if abs(event.value) > 0.1: #deadzone of joystick
               eventValues[event.axis] = event.value #storing values according to their axis
            else :
                eventValues[event.axis] = 0.0
            
            # calculating and reforming data to be sent
            x = eventValues[0]
            y = eventValues[1]

            magnitude = math.sqrt((x*x)+(y*y)) #magnitude calculated

            if x != 0:
                angle = math.atan2(y , x) #from -pi/2 ------> pi/2
                angle = math.degrees(angle)
                if angle < 0:
                    angle = angle + 360
                
                angle = angle + 90
                if angle < 0:
                    angle = angle + 360

                angle = abs(angle) % 360 #angle calculated

            if magnitude == 0 : #safety procedure
                angle = 0
            

            vector=[magnitude, angle] 
            robotcontroller.command_list(vector)#sending list to cmd receiver
            print(f'{magnitude},{angle}')
            
            #on pushing event
        if event.type == pygame.JOYBUTTONDOWN:
            if event.button == 0:   #X button -> arm down
                xpressed = ['a',-1]
                #print("X is pressed")
                robotcontroller.command_list(xpressed)

            elif event.button == 1: #circle button -> gripper open
                cpressed = ['g', 1]
                #print("C is pressed")
                robotcontroller.command_list(cpressed)

            elif event.button == 2: #square button -> gripper close
                spressed = ['g', 0]
                #print("S is pressed")
                robotcontroller.command_list(spressed)

            elif event.button == 3: #triangle button -> arm up
                tpressed = ['a',1]
                #print("T is pressed")
                robotcontroller.command_list(tpressed)


            # on releasing event
        if event.type == pygame.JOYBUTTONUP:
            if event.button == 0:
                xpressed = [False]
                #print("X is released")
                robotcontroller.command_list(xpressed)

            elif event.button == 1:
                cpressed = [False]
                #print("C is released")
                robotcontroller.command_list(cpressed)

            elif event.button == 2:
                spressed = [False]
                #print("S is released")
                robotcontroller.command_list(spressed)

            elif event.button == 3:
                tpressed = [False]
                #print("T is released")
                robotcontroller.command_list(tpressed)
    
    pygame.time.delay(10)