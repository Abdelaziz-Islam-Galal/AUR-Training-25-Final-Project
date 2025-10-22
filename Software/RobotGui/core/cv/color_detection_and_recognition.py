import cv2
import numpy as np
from PIL import Image


#------>All possible HSV colors (to mask them) until being surprised by a new color on the competetion day
COLORS_BGR = {
    'red': [20, 20, 230],
    'green': [20, 230, 20],
    'blue': [230, 20, 20],
    'yellow': [0, 230, 230],
    #'cyan': [230, 230, 0],
    #'magenta': [230, 0, 230],
    'orange': [0, 165, 230],
    'purple': [128, 0, 128],
    'pink': [203, 192, 230],
    'brown': [42, 42, 165],
    #'white': [255, 255, 255],
    #'black': [0, 0, 0],
    #'gray': [128, 128, 128],
    #'lime': [0, 230, 0],
    #'maroon': [0, 0, 128],
    #'teal': [128, 128, 0],
    #'navy': [128, 0, 0],
    #'olive': [0, 128, 128],
    #'coral': [80, 127, 255],
    #'gold': [0, 215, 255],
    #'silver': [192, 192, 192]

} 


#----> getting the lower and upper values for a color in HSV to form masks
def HSV_LowerUpper(BGRcolor):
    c = np.uint8([[BGRcolor]])  #type:ignore
    hsvC = cv2.cvtColor(c , cv2.COLOR_BGR2HSV) #type:ignore
    
    hue = hsvC[0][0][0]  

    SatThresh = 50 #for testing  
    ValThresh = 60   #for testing 

    # Handle red hue wrap-around
    if hue >= 165:  
        lowerLimit = np.array([hue - 10, SatThresh, ValThresh], dtype=np.uint8)
        upperLimit = np.array([180, 255, 255], dtype=np.uint8)
    elif hue <= 15:  
        lowerLimit = np.array([0, 100 , 100], dtype=np.uint8)

        upperLimit = np.array([hue + 10, 255, 255], dtype=np.uint8)
    else:
        lowerLimit = np.array([hue - 10, SatThresh, ValThresh], dtype=np.uint8)
        upperLimit = np.array([hue + 10, 255, 255], dtype=np.uint8)

    return lowerLimit, upperLimit


#----->mask forming function 
def FormMask(image :cv2.Mat, color:str):#image should be in HSV format
    lowerlimit = HSV_LowerUpper(COLORS_BGR[color])[0]
    upperlimit = HSV_LowerUpper(COLORS_BGR[color])[1]
    mask = cv2.inRange(image,lowerlimit,upperlimit) #

    # decreasing noise
    _, mask = cv2.threshold(mask , 0 , 255 , cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    #decreasing noise in the foreground
    kernel = np.ones((9,9),np.uint8)
    
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3,3),np.uint8))
    mask = cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
    return mask


#----->color detection class requests the image and the color 
#----->it has a property to get the x and y coordinates at which the color is saturated - for tracking
class ColorDetection():
    def __init__(self ,image , color):
        self._Frame = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
        self.length = self._Frame.shape[0]
        self.width = self._Frame.shape[1]
        self.detected = False
        self.SaturatedX = 0
        self.SaturatedY = 0
        self.mask = FormMask(self._Frame , color)#type:ignore
    
    #----->usable function to detect colors(class:ColorDetection)
    def DetectColor(self) -> bool : 
        DetectionMask = self.mask.copy()
        DetectionMask = DetectionMask[self.length//4 : 3*self.length//4, self.width//4 : 3*self.width//4]

        #using ratio to get the threshold
        self.detected = (np.sum(DetectionMask) / (DetectionMask.size * 255)) > 0.01  

        return self.detected #type:ignore
    
    @property
    def saturation(self):
        x1 = 0;x2=0;y1=0;y2=0
        SaturationMask = self.mask.copy() #copying the instance mask

        #removing small noises
        contour, _ = cv2.findContours(SaturationMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contour:
           largest = max(contour, key=cv2.contourArea)
           SaturationMask = np.zeros_like(SaturationMask)
           cv2.drawContours(SaturationMask,[largest],-1,255)
        else:
            return 0, 0

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
    
    @property
    def sat_dist_to_center(self):
        #calculating the center of the frame
        Cx = self.width/2
        Cy = self.length/2

        #calling the center of the color
        x , y = self.saturation

        #calculating the difference
        diffX = Cx-x
        diffY = Cy-y

        return diffX , diffY
    
    @property
    def right_posisiton(self):
        RightPosition = False
        diffx , diffy = self.sat_dist_to_center
        if abs(diffx) < 20 :
            RightPosition = True
        else :
            RightPosition = False
        
        return RightPosition
        

#----->usable function for color recognition(still testing)
def RecognizeColors(frame : cv2.Mat , colors):
    length = frame.shape[0]
    width = frame.shape[1]

    hsvframe = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV) 

    cx = width // 2
    cy = length // 2

    hsv = hsvframe[cy,cx]

    for colorname in colors:
       lower = HSV_LowerUpper(COLORS_BGR[colorname])[0]
       upper = HSV_LowerUpper(COLORS_BGR[colorname])[1]
       if np.all(hsv>=lower ) and np.all(hsv<=upper):
           return colorname
#       
#        
#        
    #detected = None
#
    #for color_name in colors:
    #    mask = FormMask(frame,color_name)
    #    mask = mask[mask.shape[0]//4 : 3*mask.shape[0]//4, mask.shape[1]//4 : 3*mask.shape[1]//4]
#
    #    contour , _ = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    #    if contour:
    #        largest = max(contour,key = cv2.contourArea)
    #        area = cv2.contourArea(largest)
    #        if area > 100:
    #            detected = color_name
#
    #return detected
   # pass
        

    


    
#----->testing rubbish

#cap = cv2.VideoCapture(0)
#address = "http://192.168.1.9:8080/video"
#cap.open(address)

#while True:
#    ret , frame = cap.read()
#    reco = RecognizeColors(frame , COLORS_BGR)
#    print(f'      {reco}        ')
#
#    image = ColorDetection(frame , "orange")
#    detected = image.DetectColor()
#    x , y = image.saturation
#    length = frame.shape[0]
#    width = frame.shape[1]
#    diffx , diffy = image.sat_dist_to_center
#    if detected:
#      print(f'{x},{y} ,color detected ' , {diffx} , image.right_posisiton)
#
#    cv2.imshow('frame', image.mask)
#    if cv2.waitKey(1) == ord('z'):
#        break
#
#cap.release()
#
#cv2.destroyAllWindows()