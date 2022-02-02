import cv2
import time
import numpy as np
import handTrackingModule as htm
import math
import pycaw
from pyModbusTCP.client import ModbusClient
from ctypes import cast, POINTER
from comtypes import CLSCTX_ALL
from pycaw.pycaw import AudioUtilities, IAudioEndpointVolume
import pywhatkit

pywhatkit.playonyt('animal trance')


SERVER_HOST = "192.168.0.23"
SERVER_PORT = 502
c = ModbusClient()
c.host(SERVER_HOST)
c.port(SERVER_PORT)
GRIP_addr =90 #Gripper open = 1 and close =0

##################
pTime= 0
cap = cv2.VideoCapture(0)
detector = htm.handDetector(detectionCon=0.7)
print("Attempting to connect to PLC...")


devices = AudioUtilities.GetSpeakers()
interface = devices.Activate(
IAudioEndpointVolume._iid_, CLSCTX_ALL, None)
volume = cast(interface, POINTER(IAudioEndpointVolume))
# volume.GetMute()
volume.GetMasterVolumeLevel()
volumeRange = volume.GetVolumeRange()
volume.SetMasterVolumeLevel(0, None)
minVal = volumeRange[0]
maxVal = volumeRange[1]
minVal =0
maxVal= 100
vol = 0
volBar = 400
volPer=0
################
if not c.is_open():
    if not c.open():
        print("unable to connect to " + SERVER_HOST + ":" + str(SERVER_PORT))
if c.is_open():

     while True:
          print("Connected to PLC")
          success,img= cap.read()
          img = detector.findHands(img)
          lmList = detector.findPosition(img,draw=False)
          if len(lmList) != 0:
               # print(lmList[4],lmList[8])
               x1,y1=lmList[4][1],lmList[4][2]
               x2,y2=lmList[8][1],lmList[8][2]
               cx,cy = (x1+x2)//2,(y1+y2)//2


               cv2.circle(img,(x1,y1),15,(255,0,0),cv2.FILLED)
               cv2.circle(img,(x2,y2),15,(255,0,0),cv2.FILLED)
               cv2.line(img,(x1,y1),(x2,y2),(255,0,0),3)
               cv2.circle(img,(cx,cy),15,(255,0,0),cv2.FILLED)

               length = math.hypot(x2-x1,y2-y1)
               # print(length)

               ##Hand Range 50-300
               #Volume Range -65 - 0

               vol = np.interp(length,[50,100],[minVal,maxVal])
               volBar = np.interp(length,[50,100],[400,150])
               volPer = np.interp(length,[50,100],[0,100])
               if(vol >= 70):

                    GRIP = c.write_single_register(GRIP_addr, 100)
                    GRIP = c.write_single_register(GRIP_addr, 1)
               else:
                    GRIP = c.write_single_register(GRIP_addr, 10)



               print(vol)
               volume.SetMasterVolumeLevel(vol, None)

               if length<50:
                    cv2.circle(img,(cx,cy),15,(0,255,0),cv2.FILLED)


          cv2.rectangle(img,(50,150),(85,400),(255,255,255),3)
          cv2.rectangle(img,(50,int(volBar)),(85,400),(255,255,255),cv2.FILLED)
          cv2.putText(img,f'{int(volPer)}',(40,450),cv2.FONT_HERSHEY_PLAIN,2,(255,0,0),3)




          cTime=time.time()
          fps = 1/(cTime-pTime)
          pTime = cTime


          cv2.putText(img,f'FPS; {int(fps)}',(40,70),cv2.FONT_HERSHEY_PLAIN,2,(255,0,0),3)
          cv2.imshow("Image",img)
          cv2.waitKey(1)

