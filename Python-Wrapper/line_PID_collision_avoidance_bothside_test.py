# -*- coding: utf-8 -*-
from jetbotSim import Robot, Camera
import numpy as np
import cv2
import matplotlib.pyplot as plt
import time
import torch
import torchvision
import torch.nn.functional as F
import sys


frames = 0
roadside_counter = 1

model = torchvision.models.alexnet(pretrained=False)
model.classifier[6] = torch.nn.Linear(model.classifier[6].in_features, 3)

model.load_state_dict(torch.load('best_model_20200619_bothside_test.pth'))

device = torch.device('cuda')
model = model.to(device)

mean = 255.0 * np.array([0.485, 0.456, 0.406])
stdev = 255.0 * np.array([0.229, 0.224, 0.225])

normalize = torchvision.transforms.Normalize(mean, stdev)

def preprocess(camera_value):
    global device, normalize
    x = camera_value
    x = cv2.cvtColor(x, cv2.COLOR_BGR2RGB)
    x = x.transpose((2, 0, 1))
    x = torch.from_numpy(x).float()
    x = normalize(x)
    x = x.to(device)
    x = x[None, ...]
    return x

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
    #print(red_picture)
    row_picture,col_picture=np.shape(red_picture)
    intersect_point_ratio=6/9
    return red_picture,row_picture*intersect_point_ratio

def find_intersect_point(img):
    img_red_picture,row_of_line=find_red(img)
    set_line=img_red_picture[int(row_of_line),...]
    situation=True
    if np.nonzero(set_line)[0].shape[0]==0:
        situation=False
        center_point_x=0
        car_center_point_x_left=0
        car_center_point_x_right = 0
    else:
        left_pointx=np.nonzero(set_line)[0][0]
        #print(np.nonzero(set_line)[0].shape[0])
        right_pointx=np.nonzero(set_line)[0][-1]
        center_point_x=(left_pointx+right_pointx)/2
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
        car_center_point_x_left=set_line.shape[0]/4
        car_center_point_x_right = set_line.shape[0]/(4/3)
        cv2.circle(img,(int(car_center_point_x_left),int(row_of_line)),6, (255, 160, 0), 3)
        cv2.circle(img,(int(car_center_point_x_right),int(row_of_line)),6, (255, 160, 0), 3)
        
        cv2.imshow('line_and_point',img)
        #cv2.imshow('road',img_red_picture)
    return center_point_x,car_center_point_x_left,car_center_point_x_right,situation

def execute(change):
    global robot, frames, roadside_counter
    #print("\rFrames", frames, end="")
    frames += 1
    img = cv2.resize(change["new"],(640,360))
    center_point_x,car_center_point_x_left,car_center_point_x_right,end_of_road=find_intersect_point(img)
    
    x = preprocess(img)
    y = model(x)
    
    # we apply the `softmax` function to normalize the output vector so it sums to 1 (which makes it a probability distribution)
    y = F.softmax(y, dim=1)
    
    blocked = float(y.flatten()[0])
    free = float(y.flatten()[1])
    goal = float(y.flatten()[2])
    #right = float(y.flatten()[3])
    
    detecter = np.array([free, goal, blocked])
    
    situation = np.argmax(detecter)
    
    
    situation
    #print(situation)
    print(y)
    
    if roadside_counter % 2 == 1:
        car_center_point_x = car_center_point_x_right
        
        if blocked > 0.9:
            robot.set_motor(0.2,-0.2)
            time.sleep(0.3)
            
            robot.set_motor(0.4,0.4)
            time.sleep(0.5)
            
            robot.set_motor(-0.15,0.15)
            time.sleep(0.4)
            
            robot.set_motor(0.3,0.3)
            time.sleep(0.3)
            robot.set_motor(0,0)
            
            roadside_counter = roadside_counter + 1
        
            
            print('left')
            
        elif goal > 0.85:
            robot.set_motor(0.3,0.3)
            time.sleep(0.5)
            robot.stop()
            print('GOAL!!!')
            
            sys.exit()
    
        #elif end_of_road==False:
        #    robot.stop()
        else:
            error=center_point_x-car_center_point_x
            kp=0.001
            u=kp*error
            robot.set_motor(0.25+u,0.25-u)
        
    elif roadside_counter % 2 == 0:
        car_center_point_x = car_center_point_x_left
        
        if blocked > 0.87 :
            robot.set_motor(-0.2,0.2)
            time.sleep(0.2)
            
            robot.set_motor(0.4,0.4)
            time.sleep(0.5)
            
            robot.set_motor(0.15,-0.15)
            time.sleep(0.3)
            
            robot.set_motor(0.3,0.3)
            time.sleep(0.3)
            
            
            roadside_counter = roadside_counter + 1
        
            
            print('right')
            
        elif goal > 0.85:
            robot.set_motor(0.3,0.3)
            time.sleep(0.5)
            robot.stop()
            print('GOAL!!!')
            
            sys.exit()
            
        else:
            error=center_point_x-car_center_point_x
            kp=0.001
            u=kp*error
            robot.set_motor(0.4+u,0.4-u)


robot = Robot()
camera = Camera()
camera.observe(execute)

