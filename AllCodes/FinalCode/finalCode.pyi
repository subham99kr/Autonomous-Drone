import time
import cv2
import numpy as np
import asyncio
from mavsdk import System
from mavsdk.offboard import (OffboardError, VelocityBodyYawspeed)

OX_range = [250,390]
OY_range = [180,300]
speedx=0.0
speedy=0.0
speedz=0.0
cxB=0
cyB=0
areaB=0.0
flag=0


cap=cv2.VideoCapture(0)
frameWidth = 640
frameHeight = 480
cap.set(3,frameWidth)
cap.set(4,frameHeight)

#BOX attributes
areaBRange=[]
x=[300,320]
y=[30,50]

areaDropRange=[]
OX_Drop_range = [280,360]
OY_Drop_range = [220,300]

fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output.avi',fourcc,20.0,(640,480))


lower_red=np.array([0,58,255])
upper_red=np.array([179,119,255])

#lower_yellow=np.array([0,58,255])
#upper_yellow=np.array([179,119,255])


async def run():
 drone= System()
 await drone.connect(system_address="udp://:14540")
 print("Waiting for drone to connect...")
 async for state in drone.core.connection_state():
 if state.is_connected:
 print(f"-- Connected to drone!")
 break
 
 print("-- Arming")
 await drone.action.arm()

 print("-- Setting initial setpoint")
 await drone.offboard.set_velocity_body(VelocityBodyYawspeed=(0.0, 0.0, 0.0, 0.0))
 
 print("-- Starting offboard")
 try:
 await drone.offboard.start()
 except OffboardError as error:
 print(f"Starting offboard mode failed with error code: \{error._result.result}")
 print("-- Disarming")
 await drone.action.disarm()
 return

 await drone.offboard.set_velocity_body(VelocityBodyYawspeed=(0.0, 0.0, -0.3, 0.0))
 T = time.time()+6
 while(time.time()< T): #Here I will start capturing video AND TAKEOFF to 1.8 m 
 await drone.offboard.set_velocity_body(VelocityBodyYawspeed=(0.0, 0.0, -0.3, 0.0))
 _,frame=cap.read()
 out.Write(frame)

 
 await drone.offboard.set_velocity_body(VelocityBodyYawspeed=(0.0, 0.0, 0.0, 0.0)) #After every loop i'll set it to 0.

 # move forward by 2m
 T=time.time()+5
 while(time.time()< T):
 await drone.offboard.set_velocity_body(VelocityBodyYawspeed=(0.4, 0.0, 0.0, 0.0))
 _,frame=cap.read()
 out.Write(frame)

 await drone.offboard.set_velocity_body(VelocityBodyYawspeed=(0.0, 0.0, 0.0, 0.0)) #After every loop i'll set it to 0.

 #here it will first go down to certain height(area pe depend karega)
 while True:
 _,frame=cap.read()
 hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
 mask_red=cv2.inRange(hsv,lower_red,upper_red)
 xB,yB,wB,hB = cv2.boundingRect(mask_red)
 contours_red, hierarchy_red =cv2.findContours(mask_red,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
 cxB=0
 cyB=0
 areaB=0.0
 flag=0

 if len(contours_red) !=0:
 cv2.drawContours(frame,contours_red,-1,255,3)
 c = max(contours_red,key=cv2.contourArea)
 xB,yB,wB,hB = cv2.boundingRect(mask_red)
 cxB = int(xB + wB/2)
 cyB = int(yB + hB/2)
 areaB = wB*hB
 frame= cv2.rectangle(frame,(xB,yB),(xB+wB,yB+hB),(0,255,0),2)
 cv2.circle(frame,(cxB,cyB),7,(255,255,255),-1)
 cv2.putText(frame,"Box",(cxB-20,cyB-20),cv2.FONT_HERSHEY_SIMPLEX,2.5,(255,255,255),3)
 flag=1

 cv2.imshow("Result",frame)
 out.write(frame)

 if (areaB >= areaBRange[1]):
 speedZ=0.4
 elif(areaB <= areaBRange[0]):
 speedZ=-0.4
 elif(areaBRange[0] < areaB < areaBRange[1]):
 speedZ=0.0
 
 if(cxB <= OX_range[0]):
 speedx=-0.4
 elif(cxB >= OX_range[1]):
 speedx=0.4
 elif(OX_range[0] < cxB < OX_range[1]):
 speedx=0.0 

 if(cyB <= OY_range[0]):
 speedy=-0.4
 elif(cyB >= OY_range[1]):
 speedy=0.4
 elif(OY_range[0] < cyB < OY_range[1]):
 speedy=0.0 
 
 if(flag==1):
 await drone.offboard.set_velocity_body(VelocityBodyYawspeed=(speedy, speedx, speedZ, 0.0))

 if (speedx==0 & speedy==0 & speedZ==0 & flag==1):
 break 
 
 #aab wo (x,y) pe jiaga
 while True:
 _,frame=cap.read()
 hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
 mask_red=cv2.inRange(hsv,lower_red,upper_red)
 xB,yB,wB,hB = cv2.boundingRect(mask_red)
 contours_red, hierarchy_red =cv2.findContours(mask_red,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
 cxB=0
 cyB=0
 areaB=0.0
 flag=0

 if len(contours_red) !=0:
 cv2.drawContours(frame,contours_red,-1,255,3)
 c = max(contours_red,key=cv2.contourArea)
 xB,yB,wB,hB = cv2.boundingRect(mask_red)
 cxB = int(xB + wB/2)
 cyB = int(yB + hB/2)
 areaB = wB*hB
 frame= cv2.rectangle(frame,(xB,yB),(xB+wB,yB+hB),(0,255,0),2)
 cv2.circle(frame,(cxB,cyB),7,(255,255,255),-1)
 cv2.putText(frame,"Box",(cxB-20,cyB-20),cv2.FONT_HERSHEY_SIMPLEX,2.5,(255,255,255),3)
 flag=1

 cv2.imshow("Result",frame)
 out.write(frame)
 
 if(cxB <= x[0]):
 speedx=-0.1
 elif(cxB >= x[1]):
 speedx=0.1
 elif(x[0] < cxB < x[1]):
 speedx=0.0 

 if(cyB <= y[0]):
 speedy=-0.1
 elif(cyB >= y[1]):
 speedy=0.4
 elif(y[0] < cyB < y[1]):
 speedy=0.0 
 
 if(flag==1):
 await drone.offboard.set_velocity_body(VelocityBodyYawspeed=(speedy, speedx, 0.0, 0.0))

 if (speedx==0 & speedy==0 & flag==1):
 break 

 # yaha motor niche hoga fir uper hoga


 #aab wo apna axis pe ghumte hue uthega taki face drop k taraf ho
 await drone.offboard.set_velocity_body(VelocityBodyYawspeed=(0.0, 0.0, -0.3, -30.0))
 T=time.time()+5
 while(time.time()<T):
 _,frame=cap.read()
 out.Write(frame)

 #aab wo drop a pass jiga
 await drone.offboard.set_velocity_body(VelocityBodyYawspeed=(0.4, 0.0, 0.0, 0.0))
 T = time.time()+5
 while(time.time()< T): 
 _,frame=cap.read()
 out.Write(frame)

 await drone.offboard.set_velocity_body(VelocityBodyYawspeed=(0.0, 0.0, 0.0, 0.0))

 # drop k center find kr k 1.3 m k altitude pe rahega

 while True: # 1.3 m he uper rahe (area nikalna hoga iska)
 _,frame=cap.read()
 hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
 mask_yellow=cv2.inRange(hsv,lower_yellow,upper_yellow)
 xB,yB,wB,hB = cv2.boundingRect(mask_yellow)
 contours_yellow, hierarchy_yellow =cv2.findContours(mask_yellow,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
 cxB=0
 cyB=0
 areaB=0.0
 flag=0

 if len(contours_yellow) !=0:
 cv2.drawContours(frame,contours_yellow,-1,255,3)
 c = max(contours_yellow,key=cv2.contourArea)
 xB,yB,wB,hB = cv2.boundingRect(mask_yellow)
 cxB = int(xB + wB/2)
 cyB = int(yB + hB/2)
 areaB = wB*hB
 frame= cv2.rectangle(frame,(xB,yB),(xB+wB,yB+hB),(0,255,0),2)
 cv2.circle(frame,(cxB,cyB),7,(255,255,255),-1)
 cv2.putText(frame,"Drop Zone",(cxB-20,cyB-20),cv2.FONT_HERSHEY_SIMPLEX,2.5,(255,255,255),3)
 flag=1

 cv2.imshow("Result",frame)
 out.write(frame)

 if (areaB >= areaDropRange[1]):
 speedZ=0.4
 elif(areaB <= areaDropRange[0]):
 speedZ=-0.4
 elif(areaDropRange[0] < areaB < areaDropRange[1]):
 speedZ=0.0
 
 if(cxB <= OX_Drop_range[0]):
 speedx=-0.4
 elif(cxB >= OX_Drop_range[1]):
 speedx=0.4
 elif(OX_Drop_range[0] < cxB < OX_Drop_range[1]):
 speedx=0.0 

 if(cyB <= OY_Drop_range[0]):
 speedy=-0.4
 elif(cyB >= OY_Drop_range[1]):
 speedy=0.4
 elif(OY_Drop_range[0] < cyB < OY_Drop_range[1]):
 speedy=0.0 
 
 if(flag==1):
 await drone.offboard.set_velocity_body(VelocityBodyYawspeed=(speedy, speedx, speedZ, 0.0))

 if (speedx==0 & speedy==0 & speedZ==0 & flag==1):
 break 
 
 # 0.3 m uper jao drop zone k
 await drone.offboard.set_velocity_body(VelocityBodyYawspeed=(0.0, 0.0, 0.2, 0.0))
 T = time.time()+5
 while(time.time()< T): 
 _,frame=cap.read()
 out.Write(frame) 

 # Switchoff electromagnet


 # drone ko 1.8 m k altitude pe bhejo face landing k taraf krte hue
 await drone.offboard.set_velocity_body(VelocityBodyYawspeed=(0.0, 0.0, -0.4, 20.0))
 T = time.time()+4
 while(time.time()< T): 
 _,frame=cap.read()
 out.Write(frame) 

 await drone.offboard.set_velocity_body(VelocityBodyYawspeed=(0.0, 0.0, 0.0, 0.0))

 # landing k center pe le jao
 while True: # 1.3 m he uper rahe (area nikalna hoga iska)
 _,frame=cap.read()
 hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
 mask_blue=cv2.inRange(hsv,lower_blue,upper_blue)
 xB,yB,wB,hB = cv2.boundingRect(mask_blue)
 contours_blue, hierarchy_blue =cv2.findContours(mask_blue,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
 cxB=0
 cyB=0
 areaB=0.0
 flag=0

 if len(contours_yellow) !=0:
 cv2.drawContours(frame,contours_blue,-1,255,3)
 c = max(contours_blue,key=cv2.contourArea)
 xB,yB,wB,hB = cv2.boundingRect(mask_blue)
 cxB = int(xB + wB/2)
 cyB = int(yB + hB/2)
 areaB = wB*hB
 frame= cv2.rectangle(frame,(xB,yB),(xB+wB,yB+hB),(0,255,0),2)
 cv2.circle(frame,(cxB,cyB),7,(255,255,255),-1)
 cv2.putText(frame,"Landing Zone",(cxB-20,cyB-20),cv2.FONT_HERSHEY_SIMPLEX,2.5,(255,255,255),3)
 flag=1

 cv2.imshow("Result",frame)
 out.write(frame)

 if (areaB >= areaDropRange[1]):
 speedZ=0.4
 elif(areaB <= areaDropRange[0]):
 speedZ=-0.4
 elif(areaDropRange[0] < areaB < areaDropRange[1]):
 speedZ=0.0
 
 if(cxB <= OX_Drop_range[0]):
 speedx=-0.4
 elif(cxB >= OX_Drop_range[1]):
 speedx=0.4
 elif(OX_Drop_range[0] < cxB < OX_Drop_range[1]):
 speedx=0.0 

 if(cyB <= OY_Drop_range[0]):
 speedy=-0.4
 elif(cyB >= OY_Drop_range[1]):
 speedy=0.4
 elif(OY_Drop_range[0] < cyB < OY_Drop_range[1]):
 speedy=0.0 
 
 if(flag==1):
 await drone.offboard.set_velocity_body(VelocityBodyYawspeed=(speedy, speedx, speedZ, 0.0))

 if (speedx==0 & speedy==0 & speedZ==0 & flag==1):
 break 
 
 #0.2 m/s se 4 sec k lia niche lao
 await drone.offboard.set_velocity_body(VelocityBodyYawspeed=(0.0, 0.0, 0.2, 0.0))
 T = time.time()+9
 while(time.time()< T): 
 _,frame=cap.read()
 out.Write(frame) 

 print("-- Stopping offboard")
 try:
 await drone.offboard.stop()
 except OffboardError as error:
 print(f"Stopping offboard mode failed with error code: \
 {error._result.result}") 



if __name__ == "__main__":
 loop = asyncio.get_event_loop()
 loop.run_until_complete(run())