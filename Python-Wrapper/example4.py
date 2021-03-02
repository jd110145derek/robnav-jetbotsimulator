from jetbotSim import Robot, Camera
import numpy as np
import cv2
import time
frames = 0
last_error_list= []
last_time=0
current_time=0

def getContour(input):
    _,contours,hierarchy = cv2.findContours(input, cv2.RETR_TREE, \
    								cv2.CHAIN_APPROX_SIMPLE)

    if(len(contours)>0):
    	s = max(contours,key = cv2.contourArea)
    else:
    	s = None
    

    return s

def execute(change):
    Kp = 0.0000003
    Ki = 0.0000000000003

    
    # last_time = current_time
    #sample_time =current_time- last_time
    PTerm = 0.0
    ITerm = 0.0
    last_error = 0.0
    #print(last_error)
    #Windup Guard
    int_error = 0.0
    windup_guard = 20.0
    output = 0.0
    global robot, frames, last_error_list,last_time,current_time
    current_time = time.time()
    sample_time =current_time - last_time
    print(last_error_list)
    #robot.set_motor(0.06,0.06)
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
    # Visualize
    img = cv2.resize(change["new"],(640,360))
    #opencv
    result = img.copy()
    image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    #lower = np.array([155,25,0])
    lower = np.array([155,25,0])
    upper = np.array([179,255,255])
    mask = cv2.inRange(image, lower, upper)
    result = cv2.bitwise_and(result, result, mask=mask)
    result = cv2.GaussianBlur(result, (3, 3), 0)
    black = cv2.cvtColor(result, cv2.COLOR_RGB2GRAY)
    ret,thresh= cv2.threshold(black, 50, 255, cv2.THRESH_BINARY)

    #find contour
    contour = getContour(thresh)
    #find line center
    moment = cv2.moments(contour)

    if (moment["m00"] != 0):
            #center x
        cx = int(moment["m10"]/moment["m00"])
        # #center y
        cy = int(moment["m01"]/moment["m00"])

        width = int((np.size(image, 1))/2)

    else:
        cx = 0
        cy = 0
        width = 0


    #print("width,cx,cy:",width,cx,cy)


    if cx != 0 and cy != 0 and width != 0:

        cv2.circle(img, (cx, cy), 4, (0, 255, 0), 2)  
        cv2.circle(img, (width, cy), 4, (0, 255, 255), 2) 

    print(width,cx)
    error = width - cx
    print(error)
    #current_time = time.time()
    #delta_time = current_time - last_time
    #delta_error = error - last_error
    # if (delta_time >= sample_time):
    #     PTerm = Kp * error
    #     ITerm += error * delta_time
    #     if (ITerm < -windup_guard):
    #         ITerm = -windup_guard
    #     elif(ITerm > windup_guard):
    #         ITerm = windup_guard
    # Remember last time and last error for next calculation
    last_time = current_time
    #last_error = error
    last_error_list.append(error)
    ITerm=sample_time*sum(last_error_list)
    print("last :",ITerm)
    PTerm = Kp * error
    w = PTerm+Ki*ITerm
    print("w:",w)
    if w > 0 :
        robot.set_motor(0.05,0.05)
        robot.add_motor(0,2*w)
    elif w < 0 :
        robot.set_motor(0.05,0.05)
        robot.add_motor(-2*w,0)
    else:
        robot.set_motor(0.05,0.05)

    # if abs(w) > 4 :
    #     w = 4
    # else :
    #     if w > 0 :
    #         robot.set_motor(0.05,0.05)
    #         robot.add_motor(0,w)
    #     elif w < 0 :
    #         robot.set_motor(0.05,0.05)
    #         robot.add_motor(-w,0)
    #     else : 
    #         robot.set_motor(0.05,0.05)
    cv2.imshow('result', result)
    cv2.imshow("camera", img)
    cv2.imshow("hsv", image)

robot = Robot()
camera = Camera()
camera.observe(execute)