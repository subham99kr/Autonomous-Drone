import cv2
import numpy as np

cap=cv2.VideoCapture(0)
cap.set(3,640)
cap.set(4,480)
cxB=0.0
cyB=0.0
areaB=0.0
while True:
    _,frame=cap.read()

    hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)

    lower_red=np.array([0,0,200])
    upper_red=np.array([0,0,255])

    mask_red=cv2.inRange(hsv,lower_red,upper_red)

    xB,yB,wB,hB = cv2.boundingRect(mask_red)

    contours_red, hierarchy_red =cv2.findContours(mask_red,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

    if len(contours_red) !=0:
        cv2.drawContours(frame,contours_red,-1,255,3)
        c = max(contours_red,key=cv2.contourArea)
        xB,yB,wB,hB = cv2.boundingRect(mask_red)
        cxB = int(xB + wB/2)
        cyB = int(yB + hB/2)
        areaB = wB*hB
        frame= cv2.rectangle(frame,(xB,yB),(xB+wB,yB + hB),(0,255,0),2)
        cv2.circle(frame,(cxB,cyB),7,(255,255,255),-1)
        cv2.putText(frame,"Box",(cxB-20,cyB-20),cv2.FONT_HERSHEY_SIMPLEX,2.5,(255,255,255),3)

            

    cv2.imshow("Result",frame)

    k=cv2.waitKey(5)
    if k==27:
        break

cap.release()
cv2.destroyAllWindows()