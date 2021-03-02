# -*- coding: utf-8 -*-
"""
Created on Tue Jun 16 20:22:41 2020

@author: yjuaa
"""

from jetbotSim import Robot, Camera
import numpy as np
import cv2
import time
frames = 0

def getContour(input):
    _,contours,hierarchy = cv2.findContours(input, cv2.RETR_TREE, \
    								cv2.CHAIN_APPROX_SIMPLE)

    if(len(contours)>0):
    	s = max(contours,key = cv2.contourArea)
    else:
    	s = None
    

    return s

def execute(change):
    Kp = 0.00001
    Ki = 0.001
    sample_time = 0.00
    current_time = time.time()
    last_time = current_time
    PTerm = 0.0
    ITerm = 0.0
    last_error = 0.0
    #Windup Guard
    int_error = 0.0
    windup_guard = 20.0
    output = 0.0
    global robot, frames
    print("\rFrames", frames, end="")
    frames += 1
    '''
    # Control Example
    if frames == 1:
        robot.forward(0.2)
    if frames == 80:
        robot.left(0.05)
    if frames == 88:
        robot.stop()
    if frames == 90:
        robot.set_motor(0.2,0.2)
    if frames == 200:
        robot.set_left_motor(0)
    if frames == 201:
        robot.set_right_motor(0)
    if frames == 202:
        robot.right(0.05)
    if frames == 210:
        robot.backward(0.2)
    if frames == 320:
        robot.add_motor(0,0.02)
    if frames == 400:
        robot.reset()
    '''
    
    
    
    ###road finding method 1 (20200615) 
    '''
    # Visualize
    img = cv2.resize(change["new"],(640,360))
    #opencv
    result = img.copy()
    image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower = np.array([155,25,0])
    upper = np.array([179,255,255])
    mask = cv2.inRange(image, lower, upper)
    result = cv2.bitwise_and(result, result, mask=mask)
    result = cv2.GaussianBlur(result, (3, 3), 0)
    black = cv2.cvtColor(result, cv2.COLOR_RGB2GRAY)
    ret,thresh= cv2.threshold(black, 50, 255, cv2.THRESH_BINARY)

    '''
    ###road finding method 2(20200616)
    # Visualize
    img = cv2.resize(change["new"],(640,360))

    
    (B,G,R) = cv2.split(img);
    ret,B= cv2.threshold(B, 140, 255, cv2.THRESH_BINARY_INV)
    ret,R= cv2.threshold(R, 140, 255, cv2.THRESH_BINARY)
    ret,G= cv2.threshold(G, 140, 255, cv2.THRESH_BINARY)
    
    ##use the binary image of r channel and b channel to find the track
    alpha = 0.5
    beta = (1.0 - alpha)
    merge = cv2.addWeighted(R, alpha, B, beta, 0.0)
    ret,merge= cv2.threshold(merge, 200, 255, cv2.THRESH_BINARY)
                

    #masked_black = cv2.rectangle(black, (0,0), (640,220), (0,0,0), -1)
    masked_black = cv2.rectangle(merge, (0,0), (640,220), (0,0,0), -1)
    ret,thresh= cv2.threshold(masked_black, 5, 255, cv2.THRESH_BINARY)


    #find contour
    contour = getContour(thresh)
    #find line center
    moment = cv2.moments(contour)

    if (moment["m00"] != 0):
            #center x
        cx = int(moment["m10"]/moment["m00"])
        # #center y
        cy = int(moment["m01"]/moment["m00"])

        width = int((np.size(img, 1))/2)

    else:
        cx = 0
        cy = 0
        width = 0


    #print("width,cx,cy:",width,cx,cy)


    if cx != 0 and cy != 0 and width != 0:

        cv2.circle(img, (cx, cy), 4, (0, 255, 0), 2)  
        cv2.circle(img, (width, cy), 4, (0, 255, 255), 2) 

    error = width - cx
    current_time = time.time()
    delta_time = current_time - last_time
    delta_error = error - last_error
    if (delta_time >= sample_time):
        PTerm = Kp * error
        ITerm += error * delta_time
        if (ITerm < -windup_guard):
            ITerm = -windup_guard
        elif(ITerm > windup_guard):
            ITerm = windup_guard
    # Remember last time and last error for next calculation
    last_time = current_time
    last_error = error
    w = PTerm + (Ki * ITerm)
    print("w:",w)
    if w > 0 :
        robot.set_motor(0.03,0.03)
        robot.add_motor(0,0.7*w)
    elif w < 0 :
        robot.set_motor(0.03,0.03)
        robot.add_motor(-0.7*w,0)
    else : 
        robot.set_motor(0.03,0.03)

    #cv2.imshow('result', result)
    cv2.imshow("camera", img)
    cv2.imshow("crop",masked_black)
    cv2.imshow("R",R)
    cv2.imshow("G",G)
    cv2.imshow("B",B)
    cv2.imshow("merge",merge)

robot = Robot()
camera = Camera()
camera.observe(execute)