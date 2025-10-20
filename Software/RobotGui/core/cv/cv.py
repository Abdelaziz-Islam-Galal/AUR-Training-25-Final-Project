import cv2
from threading import Thread
from time import sleep
from RobotGui.core.cv.QR_scanner import qr_scanner
from RobotGui.core.cv.detections import QR_Detector
from RobotGui.core.cv.detections import color_detector
from RobotGui.core.cv.color_detection_and_recognition import ColorDetection
import numpy as np

class Camera:
    def __init__(self) -> None:
        self._empty_frame = np.zeros((500, 500, 3), dtype=np.uint8)
        self._cap = cv2.VideoCapture()
        self._cap.open("http://192.168.1.3:8080/video")
        self._frame = None
        self._last_qr = "no QR code"
        self._frame_thread = Thread(target=self._frame_loop, daemon=True)
        self._qr_thread = Thread(target=self.qr_loop, daemon=True)
        self._frame_thread.start()
        self._qr_thread.start()
        self._detection_thread = Thread(target=self.detection_loop, daemon=True)
        self._detection_thread.start()
        self._color_thread = Thread(target=self.color_loop, daemon=True)
        self._color_thread.start()

    def _frame_loop(self):
        while True:
            success, self.image = self._cap.read()
            if success:
                self._frame = self.image
                
            else:
                self._cap.release()
                self._cap.open("http://192.168.1.3:8080/video")
            
            sleep(0.015)

    def qr_loop(self):
        while True:
            img = self._frame
            if img is not None:
                result = qr_scanner(img)
                self._last_qr = result
            sleep(0.2)            
    def detection_loop(self):
        while True:
            img = self._frame
            if img is not None:
                QR_Detector(img)
                color_detector(img)
            sleep(0.2)          
    def color_loop(self):
        while True:
            img = self._frame
            if img is not None:
                ColorDetection(img)
            sleep(0.2)   
    @property
    def frame(self):
        return self._frame if self._frame is not None else self._empty_frame
    
    @property
    def last_qr(self):
        return self._last_qr    