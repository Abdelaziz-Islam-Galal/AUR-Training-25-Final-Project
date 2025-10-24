import cv2
from RobotGui.core.cv.QR_scanner import qr_scanner
import numpy as np
from time import time
from RobotGui.core.communication.subscribe.Subscribers_methods import SubscribersMethods

def qr_auto():
    from RobotGui.gui.window import subscriber
    from RobotGui.gui.camera_display import camera_device

    x,y,theta = subscriber.coordinates()
    if y > 2:
        camera_device.start_qr_thread()
        result = camera_device.last_qr
        return result