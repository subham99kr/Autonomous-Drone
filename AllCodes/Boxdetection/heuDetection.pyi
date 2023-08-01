import cv2
import numpy as np
global xB,yB

def empty(img):
    pass

video=cv2.VideoCapture(0)
frameWidth  =  640
frameHeight =  480
video.set(3,frameWidth)
video.set(4,frameHeight)

#fourcc = cv2.VideoWriter_fourcc(*'XVID')
#out = cv2.VideoWriter('output.avi',fourcc,20.0,(640,480))
#out.write(img)

cv2.namedWindow("TrackBar")
cv2.resizeWindow("TrackBar",600,300)
cv2.createTrackbar("hue_min","TrackBar",0,179,empty)
cv2.createTrackbar("hue_max","TrackBar",179,179,empty)
cv2.createTrackbar("sat_min","TrackBar",0,255,empty)
cv2.createTrackbar("sat_max","TrackBar",255,255,empty)
cv2.createTrackbar("val_min","TrackBar",0,255,empty)
cv2.createTrackbar("val_max","TrackBar",255,255,empty)


while True:
    ret,img=video.read()
    hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    hue_min=cv2.getTrackbarPos("hue_min","TrackBar")
    hue_max=cv2.getTrackbarPos("hue_max","TrackBar")
    sat_min=cv2.getTrackbarPos("sat_min","TrackBar")
    sat_max=cv2.getTrackbarPos("sat_max","TrackBar")
    val_min=cv2.getTrackbarPos("val_min","TrackBar")
    val_max=cv2.getTrackbarPos("val_max","TrackBar")

    lower=np.array([hue_min,sat_min,val_min])  #hue_min,sat_min,val_min
    upper=np.array([hue_max,sat_max,val_max])  #hue_max,sat_max,val_max
    mask=cv2.inRange(hsv,lower,upper)
    
    
    
    cnts,hei=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    for c in cnts:
        area1=cv2.contourArea(c)
        
    if area1>5000:
        peri=cv2.arcLength(c,True)
        approx1=cv2.approxPolyDP(c,0.02*peri,True)
        x1,y1,w1,h1 = cv2.boundingRect(c)
        area1f = w1*h1
        xB = x1 + w1/2
        yB = y1 + h1/2
        cv2.rectangle(img,(x1,y1),(x1+w1,y1+h1),(0,255,0),2)
        if len(approx1)==4:
            cv2.putText(img,"BOX",(x1+w1+20,y1+h1+20),cv2.FONT_HERSHEY_COMPLEX,0.7,(0,255,0),2)
    
    
    cv2.imshow("Frame",img)
    cv2.imshow("HSV",hsv)
    cv2.imshow("Mask",mask)
    print (xB)
    k=cv2.waitKey(1)
    if k==ord('q'):
        break
video.release()
cv2.destroyAllWindows()