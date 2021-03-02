from jetbotSim import Robot, Camera
import numpy as np
import cv2
import matplotlib.pyplot as plt
import time


frames = 0

def find_red(img):
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    red1_lower=np.array([0,120,46])
    red1_uper=np.array([5,255,255])
    red2_lower=np.array([156,120,46])
    red2_uper=np.array([180,255,255])
    red_picture1=cv2.inRange(hsv_img,red1_lower,red1_uper)
    red_picture2=cv2.inRange(hsv_img,red2_lower,red2_uper)
    red_picture=cv2.bitwise_or(red_picture1,red_picture2)
    print("Success for produce red_picture")
    cv2.imshow('red_picture',red_picture)
    #plt.imshow(red_picture)
    # plt.show()
    # cv2.waitKey(0)
    print(red_picture)
    row_picture,col_picture=np.shape(red_picture)
    intersect_point_ratio=6/9
    return red_picture,row_picture*intersect_point_ratio

def find_intersect_point(img):
    img_red_picture,row_of_line=find_red(img)
    #print(type(img_red_picture))
    set_line=img_red_picture[int(row_of_line),...]
    print(type(set_line))
    situation=True
    if np.nonzero(set_line)[0].shape[0]==0:
        situation=False
        center_point_x=0
        car_center_point_x=0
    else:
        left_pointx=np.nonzero(set_line)[0][0]
        #print(np.nonzero(set_line)[0].shape[0])
        right_pointx=np.nonzero(set_line)[0][-1]
        center_point_x=(left_pointx+right_pointx)/2
        print("r:",right_pointx)
        # mask_for_line=np.zeros_like(img_red_picture)
        # mask_for_line[int(row_of_line),...]=set_line
        # cv2.imshow('center_line',mask_for_line)
        # cv2.waitKey(0)
        x1=int(left_pointx)
        y1=int(row_of_line)
        x2=int(right_pointx)
        y2=int(row_of_line)
        cv2.line(img,(x1,y1),(x2,y2),(0,255,0),3)
        cv2.circle(img,(int(center_point_x),int(row_of_line)),6, (255, 0, 0), 3)
        car_center_point_x=set_line.shape[0]/2
        cv2.circle(img,(int(car_center_point_x),int(row_of_line)),6, (255, 160, 0), 3)
        cv2.imshow('line_and_point',img)
        #cv2.imshow('road',img_red_picture)
    return center_point_x,car_center_point_x,situation

def execute(change):
    global robot, frames
    print("\rFrames", frames, end="")
    frames += 1
    img = cv2.resize(change["new"],(640,360))
    center_point_x,car_center_point_x,end_of_road=find_intersect_point(img)
    if end_of_road==False:
        robot.stop()
    else:
        error=center_point_x-car_center_point_x
        kp=0.0014
        u=kp*error
        robot.set_motor(0.3+u,0.3-u)


robot = Robot()
camera = Camera()
camera.observe(execute)


