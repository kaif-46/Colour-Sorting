import cv2
import numpy as np
from pyModbusTCP.client import ModbusClient
import time

SERVER_HOST = "192.168.0.23"
SERVER_PORT = 502
c = ModbusClient()
c.host(SERVER_HOST)
c.port(SERVER_PORT)

a=0
A=[0]
next=1
end=0
cam = False
CX_addr=50 #Coordinates for x axis
SX_addr=14 #Sign for x axis
CY_addr=51 #Coordinates for y axis
SY_addr=15 #Sign for y axis
Colour_addr = 52 #green =1 ; blue = 2; red = 3;
Ang_addr = 54 # address for orientation angle
Angsign_addr = 19 # address for orientation angle
GRIP_addr =100 #Gripper open = 1 and close =0
CTC_addr = 11 #clear to capture
CTCB_addr = 12
CTS_addr = 13
CTM_addr = 10


print("Attempting to connect to PLC...")


if not c.is_open():
    if not c.open():
        print("unable to connect to " + SERVER_HOST + ":" + str(SERVER_PORT))
if c.is_open():
    while(1):
        # start = c.read_holding_registers(CTC_addr,1)
        start = c.read_coils(11, 1)  #Y14() , Y15(D6008)
        print("Connected to PLC")
        print("waiting for start")
        print(start)
        # start = int(input("Please press 1 to start"))
        c.write_single_coil(CTS_addr, 0)
        c.write_single_coil(CTM_addr, 0)
        if cam == True:
            cap.release()
            cv2.destroyAllWindows()


        ############                    ################
        ############    Wait for start  ################
        ############                    ################

        while start[0] == 1 and a == 0:
                
                
            
            print("Capturing Image..")
            while(a == 0):
                GRIP = c.write_single_register(GRIP_addr, 10)
                A=[0]
                if cam == True:
                    cap.release()
                    cv2.destroyAllWindows()
                print("Extraction started")
                cap = cv2.VideoCapture(0) # capturing video
                _, frame = cap.read()   # capturing a frame from the video
                cam = True
                cv2.imshow('frame',frame)
            
                blue_frame = frame   # passing the captured frame to variables for further processing
                green_frame = frame
                red_frame = frame

                #save the captured frame as blue,green and red frames

                if a== 0:



                    ############                    ################
                    ############    CHECK FOR GREEN ################
                    ############                    ################
                    print("checking green")
                    hsv = cv2.cvtColor(green_frame, cv2.COLOR_BGR2HSV)   # converting the captured frame from BGR to HSV format
                    lower_green = np.array([179,0,255])  # specifying the lower threshold of HSV values for green color
                    upper_green = np.array([179,255,255])  # specifying the upper threshold of HSV values for green color
                    mask = cv2.inRange(hsv, lower_green, upper_green)  # seperating the specified colour from the image
                    res = cv2.bitwise_and(green_frame,green_frame, mask= mask)  # converting the seperated colour to white and rest of the colors to black
                    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)  # Forming contours around the detected clour
                    for contour in contours: # itterating through each contour formed
                        area = cv2.contourArea(contour) # finding out area of each formed contour
                        print(area)

                        if area < 100 : # eliminating contours with area less than threshold value
                            continue

                        cv2.drawContours(green_frame, contour, -1, (0, 255, 0), 1)  #it will mark the boundaries with colour the contour

                        rect = cv2.minAreaRect(contour)
                        box = cv2.boxPoints(rect)
                        box = np.int0(box)

                        # Retrieve the key parameters of the rotated bounding box
                        center = (int(rect[0][0]), int(rect[0][1]))
                        width = int(rect[1][0])
                        height = int(rect[1][1])
                        angle = int(rect[2])

                        if width < height:
                            angle = 90 - angle
                        else:
                            angle = - angle

                        label = "  Rotation Angle: " + str(angle) + " degrees"
                        # textbox = cv2.rectangle(blue_frame, (center[0]-35, center[1]-25),
                        #     (center[0] + 295, center[1] + 10), -1)
                        cv2.putText(green_frame, label, (center[0] - 0, center[1] - 60),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 1, cv2.LINE_AA)
                        cv2.drawContours(green_frame, [box], 0, (0, 0, 255), 2)


                        M = cv2.moments(contour)  #finding center of contour

                        if M["m00"] != 0:
                            cX = int(M["m10"] / M["m00"])
                            cY = int(M["m01"] / M["m00"])
                        else:
                            cX, cY = 0, 0
                        cv2.circle(green_frame, (cX, cY), 2, (255, 255, 255), -1)  #marking center
                        cv2.putText(green_frame, "center", (cX - 25, cY - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)  #putting text on center
                        print("green Co-ordinates")
                        print(cX)
                        print(cY)

                        s2 = (0.6484* cY) - 231.91
                        s1 = (0.6921 * cX) + 349.31 #
                        print(" green Co-or")
                        print(int(s1))
                        print(int(s2))
                        if s1 < 0:
                            s1 = (s1 * (-1))
                            c.write_single_coil(SX_addr,1)
                        else:
                            c.write_single_coil(SX_addr,0)
                        if s2 < 0:

                            s2 = (s2 * (-1))
                            c.write_single_coil(SY_addr,1)
                        else:
                            c.write_single_coil(SY_addr,0)

                        print(" green Co-or")
                        print(int(s1))
                        print(int(s2))

                        P_CY = c.write_single_register(CY_addr, int(s2))
                        P_CX = c.write_single_register(CX_addr, int(s1))
                        colour = c.write_single_register(Colour_addr, 1)

                        # GRIP = c.write_single_register(addr2, s3)


                        cv2.imshow('frame',green_frame)  #show the resultant image
                        k = cv2.waitKey(10) & 0xFF  #for display of image

                        if k == 27:
                            break

                        a=1
                        if a == 1:
                            # cap.release()  #destroy the captured camera frame
                            # cv2.destroyAllWindows() #destroy all open windows
                            break

                if a==0:


                ############                    ################
                ############    CHECK FOR BLUE  ################
                ############                    ################


                    #cap = cv2.VideoCapture(1)
                    print("checking blue")
                    #_, frame = cap.read()
                    #save green frame as org frame
                    hsv = cv2.cvtColor(blue_frame, cv2.COLOR_BGR2HSV)
                    lower_blue = np.array([11,160,135])
                    upper_blue = np.array([43,255,180])
                    mask = cv2.inRange(hsv, lower_blue, upper_blue)
                    res = cv2.bitwise_and(blue_frame,blue_frame, mask= mask)
                    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)


                    for contour in contours:
                        area = cv2.contourArea(contour)
                        if area < 100:
                            continue
                        cv2.drawContours(blue_frame, contour, -1, (0, 255, 0), 1)


                        rect = cv2.minAreaRect(contour)
                        box = cv2.boxPoints(rect)
                        box = np.int0(box)

                        # Retrieve the key parameters of the rotated bounding box
                        center = (int(rect[0][0]), int(rect[0][1]))
                        width = int(rect[1][0])
                        height = int(rect[1][1])
                        angle = int(rect[2])

                        if width < height:
                            angle = 90 - angle
                        else:
                            angle = - angle

                        #label = "  Rotation Angle: " + str(angle) + " degrees"
                        # textbox = cv2.rectangle(blue_frame, (center[0]-35, center[1]-25),
                        #     (center[0] + 295, center[1] + 10), -1)
                        label = " Center "
                        cv2.putText(blue_frame, label, (center[0] - 0, center[1] - 60),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 1, cv2.LINE_AA)
                        cv2.drawContours(blue_frame, [box], 0, (0, 0, 255), 2)




                        M = cv2.moments(contour)
                        if M["m00"] != 0:
                            cX = int(M["m10"] / M["m00"])
                            cY = int(M["m01"] / M["m00"])
                        else:
                            cX, cY = 0, 0
                        cv2.circle(blue_frame, (cX, cY), 2, (255, 255, 255), -1)
                        cv2.putText(blue_frame, "center", (cX - 25, cY - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                        print("Yellow Co-ordinates")
                        print(cX)
                        print(cY)

                        s1 = (-0.763 * cY) + 700.0 #690.0
                        s2 = (-0.770 * cX) + 233.3 #206

                        print("Robot Co-ordinates")
                        print(s1)
                        print(s2)

                        if s1 < 0:
                            s1 = (s1 * (-1))
                            c.write_single_coil(SX_addr, 1)
                        else:
                            c.write_single_coil(SX_addr, 0)
                        if s2 < 0:
                            s2 = (s2 * (-1))
                            c.write_single_coil(SY_addr, 1)
                        else:
                            c.write_single_coil(SY_addr, 0)

                        P_CY = c.write_single_register(CY_addr, int(s2))
                        P_CX = c.write_single_register(CX_addr, int(s1))
                        # GRIP = c.write_single_register(GRIP_addr, 1)
                        colour = c.write_single_register(Colour_addr, 2)

                        # print("yellow Co-ordinates")
                        # print(s1)
                        # print(s2)
                        time.sleep(1)
                        cv2.imshow('frame', blue_frame)
                        k = cv2.waitKey(10) & 0xFF

                        if k == 27:
                            break
                        a=1
                        if a == 1:
                            # cap.release()
                            # cv2.destroyAllWindows()
                            break
                    if a == 1:

                        break


                if a==0:


            ############                    ################
            ############    CHECK FOR RED   ################
            ############                    ################


                    #cap = cv2.VideoCapture(1)
                    #_, frame = cap.read()
                    print("checking red")
                    hsv = cv2.cvtColor(red_frame, cv2.COLOR_BGR2HSV)
                    lower_red1 = np.array([0,144,0])
                    upper_red1 = np.array([6,255,255])
                    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
                    lower_red2 = np.array([170,120,70])
                    upper_red2 = np.array([180,255,255])
                    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
                    # mask = mask1+mask2
                    mask = mask1
                    res = cv2.bitwise_and(red_frame, red_frame, mask=mask)
                    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

                    for contour in contours:
                        area = cv2.contourArea(contour)
                        if area < 100:
                            continue
                        cv2.drawContours(red_frame, contour, -1, (0, 255, 0), 1)

                        rect = cv2.minAreaRect(contour)
                        box = cv2.boxPoints(rect)
                        box = np.int0(box)

                        # Retrieve the key parameters of the rotated bounding box
                        center = (int(rect[0][0]), int(rect[0][1]))
                        width = int(rect[1][0])
                        height = int(rect[1][1])
                        angle = int(rect[2])

                        if width < height:
                            angle = 90 - angle
                        else:
                            angle = - angle

                        #label = "  Rotation Angle: " + str(angle) + " degrees"
                        # textbox = cv2.rectangle(blue_frame, (center[0]-35, center[1]-25),
                        #     (center[0] + 295, center[1] + 10), -1)
                        label = " Center "
                        cv2.putText(blue_frame, label, (center[0] - 0, center[1] - 60),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 1, cv2.LINE_AA)
                        cv2.drawContours(blue_frame, [box], 0, (0, 0, 255), 2)


                        M = cv2.moments(contour)
                        if M["m00"] != 0:
                            cX = int(M["m10"] / M["m00"])
                            cY = int(M["m01"] / M["m00"])
                        else:
                            cX, cY = 0, 0
                        cv2.circle(red_frame, (cX, cY), 2, (255, 255, 255), -1)
                        cv2.putText(red_frame, "center", (cX - 25, cY - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                        print("Red Co-ordinates")
                        print(cX)
                        print(cY)
                        s1 = (-0.763 * cY) + 700.0
                        s2 = (-0.770 * cX) + 233.3
                        #s2=135
                        s3 = 1
                        print("Robot Co-ordinates")
                        print(s1)
                        print(s2)

                        if s1 < 0:
                            s1 = (s1 * (-1))
                            c.write_single_coil(SX_addr, 1)
                        else:
                            c.write_single_coil(SX_addr, 0)
                        if s2 < 0:
                            s2 = (s2 * (-1))
                            c.write_single_coil(SY_addr, 1)
                        else:
                            c.write_single_coil(SY_addr, 0)

                        P_CY = c.write_single_register(CY_addr, int(s2))
                        P_CX = c.write_single_register(CX_addr, int(s1))
                        # GRIP = c.write_single_register(GRIP_addr, 1)
                        colour = c.write_single_register(Colour_addr, 3)
                        # print("Red Co-ordinates")
                        # print(s1)
                        # print(s2)
                        time.sleep(1)
                        cv2.imshow('frame', red_frame)
                        k = cv2.waitKey(10) & 0xFF

                        if k == 27:

                            break
                        a=1

                        if a == 1:
                            # cap.release()
                            # cv2.destroyAllWindows()
                            break
                    if a == 1:
                        break
                    cap.release()
                    cv2.destroyAllWindows()
            print("rotation angle")
            print(angle)
            Rangle = 90 - angle
            Rangle = -45 + Rangle
            print("robot angle")
            print(Rangle)
            if Rangle < 0:
                Rangle = (Rangle * (-1))
                print("NEGATIVE")
                c.write_single_coil(Angsign_addr, 1)
            else:
                c.write_single_coil(Angsign_addr, 0)
            c.write_single_register(Ang_addr, int(Rangle))
            c.write_single_coil(CTS_addr,1)
            time.sleep(2)
            c.write_single_coil(CTM_addr, 1)
            start = c.read_coils(11, 1)



            while(start[0]==1):
                #GRIP = c.write_single_register(GRIP_addr, 100)

                a=0

                time.sleep(2)
                #GRIP = c.write_single_register(GRIP_addr, 1)
                print("Waiting for robots command")
                start = c.read_coils(11, 1)
                c.write_single_coil(CTM_addr, 0)


                # A = input("Press 1 if object is removed")
                
                
                

                cap.release()
                cv2.destroyAllWindows()



