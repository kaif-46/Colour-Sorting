import cv2
import numpy as np
 # from pyModbusTCP.client import ModbusClient
import time

a=0
A=[0]
next=1
end=0
start = int(input("Please press 1 to start"))

############                    ################
############    Wait for start  ################
############                    ################

while start == 1 and end == 0:
           
        
    
    print("Entering 1st loop")
    while(1):
        A=[0]
        # cap.release()
        # cv2.destroyAllWindows()
        
        # next = int(input("Press 1 to start next cycle"))
        # print("next")
        # print(next)
        # while next == 1:
        print("Extraction started")
        cap = cv2.VideoCapture(1) # capturing video
        _, frame = cap.read()   # capturing a frame from the video
    
        cv2.imshow('frame',frame)
    
        blue_frame = frame   # passing the captured frame to variables for further processing
        green_frame = frame
        red_frame = frame

        #save the captured frame as blue,green and red frames
        
        ############                    ################
        ############    CHECK FOR GREEN ################
        ############                    ################
        print("checking green")
        hsv = cv2.cvtColor(green_frame, cv2.COLOR_BGR2HSV)   # converting the captured frame from BGR to HSV format
        lower_green = np.array([50, 50, 50])  # specifying the lower threshold of HSV values for green color
        upper_green = np.array([70, 255, 255])  # specifying the upper threshold of HSV values for green color
        mask = cv2.inRange(hsv, lower_green, upper_green)  # seperating the specified colour from the image
        res = cv2.bitwise_and(green_frame,green_frame, mask= mask)  # converting the seperated colour to white and rest of the colors to black
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)  # Forming contours around the detected clour
        for contour in contours: # itterating through each contour formed
            area = cv2.contourArea(contour) # finding out area of each formed contour
            if area < 500: # eliminating contours with area less than threshold value
                continue
            cv2.drawContours(green_frame, contour, -1, (0, 255, 0), 1)  #it will mark the boundaries with colour the contour

            M = cv2.moments(contour)  #finding center of contour
            
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
            else:
                cX, cY = 0, 0
            cv2.circle(green_frame, (cX, cY), 2, (255, 255, 255), -1)  #marking center
            cv2.putText(green_frame, "center", (cX - 25, cY - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)  #putting text on center
            s1=480-cY  #cordinates for the robot with eliminated offset
            s2=640-cX
            print(" green Co-or")
            print(s1)
            print(s2)

            cv2.imshow('frame',green_frame)  #show the resultant image
            k = cv2.waitKey(10) & 0xFF  #for display of image

            if k == 27:
                break

            a=1
            if a == 1:
                # cap.release()  #destroy the captured camera frame
                # cv2.destroyAllWindows() #destroy all open windows
                break
            


    ############                    ################
    ############    CHECK FOR BLUE  ################
    ############                    ################


        #cap = cv2.VideoCapture(1)

        #_, frame = cap.read()
        #save green frame as org frame
        hsv = cv2.cvtColor(blue_frame, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([102,153,0])
        upper_blue = np.array([122,255,255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        # cv2.imshow('mask',mask)
        res = cv2.bitwise_and(blue_frame,blue_frame, mask= mask)
        # cv2.imshow('f',res)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)


        for contour in contours:
            area = cv2.contourArea(contour)
            print(area)
            if area < 500:
                continue
            cv2.drawContours(blue_frame, contour, -1, (0, 255, 0), 1)


            rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            
            # Retrieve the key parameters of the rotated bounding box
            center = (int(rect[0][0]),int(rect[0][1])) 
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
            cv2.putText(blue_frame, label, (center[0]-0, center[1]-60), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,0), 1, cv2.LINE_AA)
            cv2.drawContours(blue_frame,[box],0,(0,0,255),2)
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
            else:
                cX, cY = 0, 0
            cv2.circle(blue_frame, (cX, cY), 2, (255, 255, 255), -1)
            cv2.putText(blue_frame, "center", (cX - 25, cY - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            s1=480-cY
            s2=640-cX
            addr = 0
            addr1 = 2
            addr2= 5
            s3 = 2
            # is_ok = c.write_single_register(addr, s1)
            # is_ok_yrr = c.write_single_register(addr1, s2)
            # is_ok_too_yrr = c.write_single_register(addr2, s3)
            print("Blue Co-ordinates")
            print(s1)
            print(s2)
            time.sleep(1)
            # cv2.imshow('f',res)
            # cv2.imshow('mask',mask)
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


############                    ################
############    CHECK FOR RED   ################
############                    ################


        #cap = cv2.VideoCapture(1)
        #_, frame = cap.read()
        hsv = cv2.cvtColor(red_frame, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([0,116,0])
        upper_red1 = np.array([7,255,255])
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
            if area < 500:
                continue
            cv2.drawContours(red_frame, contour, -1, (0, 255, 0), 1)
           
           
            rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            
            # Retrieve the key parameters of the rotated bounding box
            center = (int(rect[0][0]),int(rect[0][1])) 
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
            cv2.putText(blue_frame, label, (center[0]-0, center[1]-60), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,0), 1, cv2.LINE_AA)
            cv2.drawContours(red_frame,[box],0,(0,0,255),2)

            M = cv2.moments(contour)

            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
            else:
                cX, cY = 0, 0
            cv2.circle(red_frame, (cX, cY), 2, (255, 255, 255), -1)
            cv2.putText(red_frame, "center", (cX - 25, cY - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            s1 = 480 - cY
            s2 = 640 - cX
            addr = 0
            addr1 = 2
            addr2 = 5
            s3 = 3
            # is_ok = c.write_single_register(addr, s1)
            # is_ok_yrr = c.write_single_register(addr1, s2)
            # is_ok_too_yrr = c.write_single_register(addr2, s3)
            print("Red Co-ordinates")
            print(s1)
            print(s2)
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


        # cap.release()
        # cv2.destroyAllWindows()
    while(A==[0]):
        A = input("Press 1 if object is removed")
        a=0
        
        print("Waiting for robots command")
        cap.release()
        cv2.destroyAllWindows()



