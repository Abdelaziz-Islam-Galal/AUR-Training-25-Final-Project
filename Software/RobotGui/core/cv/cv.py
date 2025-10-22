import cv2
from threading import Thread
from time import sleep
from RobotGui.core.cv.QR_scanner import qr_scanner
from RobotGui.core.cv.color_detection_and_recognition import ColorDetection,COLORS_BGR,FormMask
import numpy as np

cv2.setLogLevel(0)


class Camera:
    def __init__(self) -> None:
        self._empty_frame = np.zeros((500, 500, 3), dtype=np.uint8)
        self._cap = cv2.VideoCapture() # put ip
        self._cap.open(0,cv2.CAP_DSHOW)
        self._frame = self._empty_frame
        self._last_qr = "no QR code"
        self._frame_thread = Thread(target=self._frame_loop, daemon=True)
        self._qr_thread = None
        self._frame_thread.start()
        #self._detection_thread = Thread(target=self.detection_loop, daemon=True)
        #self._detection_thread.start()
        self._color_thread = Thread(target=self.color_loop, daemon=True)
        self._color_thread.start()
        ##self.initialized=False
    def _frame_loop(self):
        while True:
            success, self.image = self._cap.read()
            if success:
                self._frame = self.image
                
            else:
                self._cap.release()
                sleep(0.01)
                self._cap.open(0,cv2.CAP_DSHOW)
                #self._cap.open("http://192.168.1.3:8080/video")

    def qr_loop(self):
            while True:
                img = self._frame.copy()
                if img is not None:
                    result = qr_scanner(img)
                    if result is not None and result != 'no QR code':
                        self._last_qr = result
                        print(self.last_qr)
                        break
                sleep(0.1)
            print("QR scanning thread stopped.")

    #def detection_loop(self):
    #    while True:
    #        img = self._frame
    #        if img is not None:
    #            QR_Detector(img)
    #            color_detector(img)
    #        sleep(0.2)          
    def color_loop(self):
        detector = None
        while True:
            img = self._frame
            if img is not None:
                if detector is None:
                    detector = ColorDetection(img, 'green')  # create once
                else:
                    detector._Frame = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
                    detector.mask = FormMask(detector._Frame, 'green')

                detected = detector.DetectColor()
                if detected:
                    pos=detector.right_posisiton
                    if pos!="no object detected":
                        print(pos)
            sleep(0.05)    

    def start_qr_thread(self):
        if self._qr_thread is None or not self._qr_thread.is_alive():
            self._qr_thread = Thread(target=self.qr_loop, daemon=True)
            self._qr_thread.start()
            print("QR scanning started.")
        else:
            print("QR thread already running.")

    @property
    def frame(self): 
        #ranges=HSV_LowerUpper(COLORS_BGR['green'])
        #frame,self.initialized,self.tracker=color_detector(self._frame,ranges,self.initialized,self.tracker,False) 
        #return frame if frame is not None else self._empty_frame
        return self._frame if self._frame is not None else self._empty_frame


    @property
    def last_qr(self):
        return self._last_qr    
    