import cv2

class Camera:
    def __init__(self) -> None:
        self._vid = cv2.VideoCapture(0)

    @property
    def frame(self):
        retval, rframe = self._vid.read()
        if retval:
            return rframe