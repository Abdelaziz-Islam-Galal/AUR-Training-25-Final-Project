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
            frame.start_qr_thread()
            result = frame.last_qr
            return result

    



