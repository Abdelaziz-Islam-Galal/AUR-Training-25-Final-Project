import cv2
from threading import Thread
from time import sleep, time
from RobotGui.core.cv.QR_scanner import qr_scanner
from RobotGui.core.cv.color_detection_and_recognition import ColorDetection,COLORS_BGR,FormMask
import numpy as np

cv2.setLogLevel(0)


class auto:
    def __init__(self) -> None:
        self._empty_frame = np.zeros((500, 500, 3), dtype=np.uint8)
        self._cap = cv2.VideoCapture() # put ip
        self._cap.open("http://192.168.1.12:8080/video")
        self._frame = self._empty_frame
        self._last_qr = "no QR code"
        self._frame_thread = Thread(target=self._frame_loop, daemon=True)
        self._qr_thread = None
        self._frame_thread.start()
        self._qr_thread = Thread(target=self.qr_detect, daemon=True)
        self._qr_thread.start()

    def _frame_loop(self):
        while True:
            success, self.image = self._cap.read()
            if success:
                self._frame = self.image
                
            else:
                self._cap.release()
                sleep(0.01)
                #self._cap.open(0,cv2.CAP_DSHOW)
                self._cap.open("http://192.168.1.12:8080/video")

    def qr_detect(self):
            img = self._frame.copy()
            detector = ColorDetection(img, 'blue')
            self.detected = detector.DetectColor()
            while True:
                 if self.detected:
                    qr_scanner()
                    sleep(0.1)    

    def qr_loop(self):
            start = time()
            duration = 10
            while time() - start < duration:
                img = self._frame.copy()
                if self.detected is not None:
                    self.start_qr()
                    result = qr_scanner(img)
                    if result is not None and result != 'no QR code':
                        self._last_qr = result
                        print(self.last_qr)
                        self.grab_box()
                        break
            print("QR scanning thread stopped.")

    def grab_box(self):
        print("box grabed")
        
             
    #def upper_part(img):
    #    height, width, _ = img.shape

    #    # Take only the top half of the frame
    #    upper_part = img[:height // 2, :]
    #    return upper_part

    @property
    def frame(self): 
        #ranges=HSV_LowerUpper(COLORS_BGR['green'])
        #frame,self.initialized,self.tracker=color_detector(self._frame,ranges,self.initialized,self.tracker,False) 
        #return frame if frame is not None else self._empty_frame
        return self._frame if self._frame is not None else self._empty_frame


    @property
    def last_qr(self):
        return self._last_qr    
    



