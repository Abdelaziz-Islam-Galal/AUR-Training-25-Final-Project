import cv2
import numpy as np
from PIL import Image


#------>All possible BGR colors (to mask them) until being surprised by a new color on the competetion day
COLORS_BGR = {
    'red': np.array([0, 0, 255]),
    'green': np.array([0, 255, 0]),
    'blue': np.array([255, 0, 0]),
    'yellow': np.array([0, 255, 255]),
    'orange': np.array([0, 165, 255]),
    'white': np.array([255, 255, 255]),
    'black': np.array([0, 0, 0]),
    'violet': np.array([238, 130, 238]),
    'indigo': np.array([130, 0, 75]),
    'cyan': np.array([255, 255, 0]),
    'magenta': np.array([255, 0, 255]),
    'purple': np.array([128, 0, 128]),
    'pink': np.array([203, 192, 255]),
    'brown': np.array([42, 42, 165]),
    'gray': np.array([128, 128, 128])
}


#----> getting the lower and upper values for a color in HSV to form masks
def BGR2HSV_LowerUpper(BGRcolor):
    c = np.uint8([[BGRcolor]])  #type:ignore
    hsvC = cv2.cvtColor(c, cv2.COLOR_BGR2HSV) #type:ignore

    hue = hsvC[0][0][0]  

    # Handle red hue wrap-around
    if hue >= 165:  
        lowerLimit = np.array([hue - 10, 100, 100], dtype=np.uint8)
        upperLimit = np.array([180, 255, 255], dtype=np.uint8)
    elif hue <= 15:  
        lowerLimit = np.array([0, 100, 100], dtype=np.uint8)
        upperLimit = np.array([hue + 10, 255, 255], dtype=np.uint8)
    else:
        lowerLimit = np.array([hue - 10, 100, 100], dtype=np.uint8)
        upperLimit = np.array([hue + 10, 255, 255], dtype=np.uint8)

    return lowerLimit, upperLimit

#----->mask forming function 
def FormMask(image :cv2.Mat, color:str):#image should be in HSV format
    lowerlimit , upperlimit = BGR2HSV_LowerUpper(COLORS_BGR[color])
    mask = cv2.inRange(image,lowerlimit,upperlimit)
    return mask


#----->color detection class requests the image and the color 
#----->it has a property to get the x and y coordinates at which the color is saturated - for tracking
class ColorDetection():
    def __init__(self ,image , color):
        self._Frame = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
        self.detected = False
        self.SaturatedX = 0
        self.SaturatedY = 0
        self.mask = FormMask(self._Frame , color)#type:ignore
    
    #----->usable function to detect colors(class:ColorDetection)
    def DetectColor(self) -> bool : 
        DetectionMask = self.mask.copy()
        self.detected = np.sum(DetectionMask) > (700 * 255)  # 500 is the threshold that may be changed upon trials in real areas
        return self.detected #type:ignore
    
    @property
    def saturation(self):
        x1 = 0;x2=0;y1=0;y2=0
        SaturationMask = self.mask.copy() #copying the instance mask

        PilFormatedMask = Image.fromarray(SaturationMask) #formating to PIL
        ColorBox = PilFormatedMask.getbbox() #getting the coords of the frame

        if ColorBox is not None:
            x1, y1, x2, y2 = ColorBox
            
        
        else :
            x1 = 0;x2=0;y1=0;y2=0
          
        #calculations to get the center
        self.SaturatedX = (x1+x2)/2 #type:ignore
        self.SaturatedY = (y1+y2)/2 #type:ignore

        return self.SaturatedX , self.SaturatedY



#----->usable function for color recognition
def RecognizeColors():
    ...#still working on code .......
    pass


    
#----->testing rubbish

#cap = cv2.VideoCapture(0)
#
#while True:
#    ret , frame = cap.read()
#
#    image = ColorDetection(frame , "green")
#    detected = image.DetectColor()
#    x , y = image.saturation
#    if detected:
#        print(f'{x},{y} ,color detected')
#
#    cv2.imshow('frame', frame)
#    if cv2.waitKey(1) == ord('z'):
#        break
#
#cap.release()
#
#cv2.destroyAllWindows()