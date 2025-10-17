import cv2
import numpy as np
#frame taken from ESP heart beat
#tracker = cv2.TrackerCSRT_create() ------>must be called before loop of detection in the main program

def QR_Detector(frame:cv2.Mat,tracker:cv2.Tracker,initialized=False):

    if initialized:
        ok,bbox=tracker.update(frame)
        if ok:
            x, y, w, h = [int(v) for v in bbox]
            x_center=x+(w//2)
            frame_x_center=frame.shape[0]//2
            if (x_center-frame_x_center)>10:
                print('move left wheel')
            elif (x_center-frame_x_center)<-10:
                print('move right wheel')
        else:   #tracking lost
            initialized=False
        return None

    else:

        img_gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        _,thresh=cv2.threshold(img_gray,127,255,cv2.THRESH_BINARY_INV)
        contours,hierarchy=cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        if hierarchy is None:
            return None
        for i,cnt in enumerate(contours) :
            real_area=cv2.contourArea(cnt)
            x1,y1,w,h=cv2.boundingRect(cnt)
            calculated_area=w*h
            if real_area>1000 and (abs(calculated_area-real_area)/real_area)<0.02: #min area for the box will be calculated 
                #hierarchy index points to one from these [Next, Previous, First_Child, Parent]
                #if it's -1 so it is a parent so it's not inside another rectangle
                parent_idx = hierarchy[0][i][3]
                if parent_idx != -1:    #this may be the qr code
                    tracker.init(frame,(x1,y1,w,h))
                    initialized=True                
                    cropped=frame[y1:y1+h,x1:x1+w]  #[height,width]
                    return cropped
        return None #if the output is not None call the qr reader function 




def color_detector(frame:cv2.Mat,lower1:np.array,upper1:np.array,lower2:np.array,upper2:np.array,tracker:cv2.Tracker):
    
    if initialized:
        ok,bbox=tracker.update(frame)
        if ok:
            x, y, w, h = [int(v) for v in bbox]
            x_center=x+(w//2)
            frame_x_center=frame.shape[0]//2
            if (x_center-frame_x_center)>10:
                print('move left wheel')
            elif (x_center-frame_x_center)<-10:
                print('move right wheel')
        else:   #tracking lost
            initialized=False
    else:
        center=(frame.shape[1]//2,frame.shape[0]//2)
        img=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(img, lower1, upper1)
        mask2 = cv2.inRange(img, lower2, upper2)
        mask = cv2.bitwise_or(mask1, mask2)
        

        contours,hierarchy=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        if hierarchy is None:
            return None
        
        if contours:
            biggest=max(contours,key=cv2.contourArea)
            if cv2.contourArea(biggest)>100:
                x1,y1,w,h=cv2.boundingRect(biggest)
                coordinates=(x1+(w//2),y1+(h//2))
                return coordinates,center
        return None
    
            


