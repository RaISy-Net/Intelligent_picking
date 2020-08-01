import os
import pdb
import cv2
import time
import glob
import random
import numpy as np
import pybullet as p
import pybullet_data
import distutils.dir_util
from pkg_resources import parse_version
from robot import robot
from drop_area import *
from run_offline import predict_grasp_angle

Area_Halfdim=1

def get_grasp_prediction(x,y,z,a):
    Robot.rgbd_images(x,y,z)
    network = "./trained models/epoch_30_iou_0.97"
    #network = "C:/Users/yashs/OneDrive/Documents/GitHub/Intelligent_picking/Phase_2_codebase/trained models/epoch_00_iou_0.93"
    rgb_path = "./CapturedImg/color"+".png"
    depth_path = "./CapturedImg/depth"+".png"
    gs = predict_grasp_angle(network, rgb_path, depth_path)
    return gs

def get_real_world_coord():
    end_effector_initpos = Robot.end_effector()[0]
    a=0
    gs = get_grasp_prediction(end_effector_initpos[0],end_effector_initpos[1],end_effector_initpos[2],a)
    x = end_effector_initpos[0]
    y = end_effector_initpos[1]
    z = end_effector_initpos[2]
    a = 0.001
    y = y - a*(1-gs[0].center[0]/112)
    x = x + a*(1-gs[0].center[1]/112)
    angle = gs[0].angle
    print(x,y)
    return x,y,angle

def pick(xpos, ypos):
    # flag to be activated by the score
    grabScore = 0.9
    Robot.move_frame_and_head(ypos+0.06, xpos-0.03)
    x,y,angle = get_real_world_coord()
    
    if(grabScore >= 0.5):
        Robot.rotate_gripper(angle)
    else:
        Robot.move_frame_and_head(ypos+0.06, xpos-0.03-0.135)

    #Robot.move_frame(ypos+0.06)
    #Robot.move_head(xpos-0.03)
    z_init = Robot.end_effector()[0][2]
    print(z_init)


    
    print('rotating gripper')
    Robot.extend_wrist(0.02)
    
    z_init = Robot.end_effector()[0][2]
    zpos = z_init - 1.06
    Robot.extend_wrist(zpos)
    Robot.close_gripper(0.10)
    Robot.contract_wrist(0.13)

def place(xpos, ypos):
    Robot.move_frame_and_head(ypos+0.06, xpos-0.03)
    #Robot.move_frame(ypos+0.06)
    #Robot.move_head(xpos-0.03)
    Robot.extend_arm()
    #Robot.extend_wrist(0.05)
    Robot.open_gripper()
    Robot.reset_gripper()
    Robot.contract_arm()

x = -0.8
y = 0.4

Robot = robot()
# for i in range(10):
#     Robot.reset(i)
p.resetDebugVisualizerCamera(2 , 0, -41, [0, -1.4, 1])
Robot.suction_up()
object_indices = [1, 5, 13, 16, 21]   #to select which object to go to
count=0
placing = [[-0.4, 0.8], [0, 0.4], [0.4, 0.12], [0.8, 0.8]]
time.sleep(2)
for i in  range(25): #can use object indices as well (to select particular object)
    print("yahan jaa raha mai")
    print(i)
    object = Robot._objectUids[i]
    pos = p.getBasePositionAndOrientation(object)[0]  
    pick(pos[0], pos[1])
    # for j in range(181):
    #     p.resetDebugVisualizerCamera(2, j, -41, [0, -1.4+(2.8*j)/180, 1-j*0.8/180])
    #     time.sleep(0.01)
    place(x, y)
    # for j in range(181):
    #     p.resetDebugVisualizerCamera(2 , 180-j, -41, [0, 1.4 - (2.8*j)/180, 0.2 + j*0.8/180])
    #     time.sleep(0.01)
    x = placing[count][0]
    y = placing[count][1]
    count+=1

time.sleep(10)

