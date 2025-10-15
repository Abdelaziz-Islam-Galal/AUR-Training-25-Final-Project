import cv2
#frame taken from ESP heart beat
def QR_Detector(frame:cv2.Mat):
    img_gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    _,thresh=cv2.threshold(img_gray,127,255,cv2.THRESH_BINARY_INV)
    contours,hierarchy=cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    if hierarchy is None:
        return None
    for i,cnt in enumerate(contours) :
        real_area=cv2.contourArea(cnt)
        x1,y1,w,h=cv2.boundingRect(cnt)
        calculated_area=w*h
        if (abs(calculated_area-real_area)/real_area)<0.02 and real_area>1000: #min area for the box will be calculated 
            #hierarchy index points to one from these [Next, Previous, First_Child, Parent]
            #if it's -1 so it is a parent so it's not inside another rectangle
            parent_idx = hierarchy[0][i][3]

            if parent_idx != -1:    #this may be the qr code
                cropped=frame[y1:y1+h,x1:x1+w]  #[height,width]
                return cropped
    return None

#if the output is not None call the qr reader function 
