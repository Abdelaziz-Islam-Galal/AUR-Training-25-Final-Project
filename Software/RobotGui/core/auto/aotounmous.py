import cv2
from RobotGui.core.cv.QR_scanner import qr_scanner
import numpy as np
from time import time
from RobotGui.core.communication.subscribe.Subscribers_methods import SubscribersMethods
from RobotGui.core.cv.cv import Camera

def qr_auto():

    coords = SubscribersMethods()
    x,y,theta = coords.coordinates()
    frame = Camera() 
    while y > 2:
        img = frame.frame()
        start = time()
        duration = 10
        while time() - start < duration:
                result = qr_scanner(img)
                return result

    



