import pygame
import math
from RobotGui.core.control.robot_controller import RobotController

robotcontroller = RobotController()

pygame.init()
pygame.joystick.init()

joystick = pygame.joystick.Joystick(0)
joystick.init()
eventValues=[0.0,0.0,0.0,0.0,0.0]

magnitude = 0;angle = 0;xpressed = False;cpressed = False;spressed = False;tpressed = False

while True:
    for event in pygame.event.get():
        if event.type == pygame.JOYAXISMOTION:
            if event.value > 0.1:
               eventValues[event.axis] = event.value
            else :
                eventValues[event.axis] = 0.0
            x = eventValues[0]
            y = eventValues[1]

            magnitude = ((x*x)+(y*y)) // 2
            if x != 0:
                angle = math.atan(y / x) #from -pi/2 ------> pi/2

            vector=[magnitude, angle] 
            robotcontroller.choose_mode(vector)
            
        if event.type == pygame.JOYBUTTONDOWN:
            if event.button == 0:
                xpressed = [True]
                robotcontroller.choose_mode(xpressed)

            if event.button == 1:
                cpressed = [True]
                robotcontroller.choose_mode(cpressed)

            if event.button == 2:
                spressed = [True]
                robotcontroller.choose_mode(spressed)

            if event.button == 3:
                tpressed = [True]
                robotcontroller.choose_mode(tpressed)
            
        if event.type == pygame.JOYBUTTONUP:
            if event.button == 0:
                xpressed = [False]
                robotcontroller.choose_mode(xpressed)

            if event.button == 1:
                cpressed = [False]
                robotcontroller.choose_mode(cpressed)

            if event.button == 2:
                spressed = [False]
                robotcontroller.choose_mode(spressed)

            if event.button == 3:
                tpressed = [False]
                robotcontroller.choose_mode(tpressed)
