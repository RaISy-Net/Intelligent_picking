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
    network = "./trained models/epoch_17_iou_0.96"
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
    a = 0.005
    y = y - a*(1-gs[0].center[0]/112)
    x = x + a*(1-gs[0].center[1]/112)
    score = 1.5
    angle = gs[0].angle
    print(x,y)
    return x,y,angle,score

def pick(xpos, ypos, object, threshold=1):
    Robot.move_frame_and_head(ypos+0.06, xpos-0.03)
    z_init = Robot.end_effector()[0][2]
    x,y,angle,score = get_real_world_coord()
    if score<threshold:
        if z_init>1.1859:
            zpos = z_init - 1.1859
            Robot.extend_wrist(zpos)
        Robot.move_suction_cup(ypos, xpos)
        Robot.suction_down()
        cons = Robot.suction_force(object)
        Robot.suction_up()
        if z_init>1.1859:
            zpos = z_init - 1.1859
            Robot.contract_wrist(zpos)
        return 1, cons
    else:
        Robot.rotate_gripper(angle)
        Robot.move_frame_and_head(y+0.06, x-0.03)
        z_init = Robot.end_effector()[0][2]
        zpos = z_init - 1.055
        Robot.extend_wrist(zpos)
        Robot.close_gripper(0.10)
        Robot.contract_wrist(0.13)
        return 0, None

def place(xpos, ypos, suction, cons):
    if suction:
        Robot.move_suction_cup(ypos, xpos)
        Robot.extend_arm()
        Robot.remove_suction_force(cons)
        Robot.contract_arm()
        Robot.reset_gripper()
    else:
        Robot.move_frame_and_head(ypos+0.06, xpos-0.03)
        Robot.extend_arm()
        Robot.open_gripper()
        Robot.reset_gripper()
        Robot.contract_arm()
        Robot.reset_gripper()

x = -0.8
y = 0.4

Robot = robot()
p.resetDebugVisualizerCamera(2 , 0, -41, [0, -1.4, 1])
Robot.suction_up()
object_indices = [1, 5, 13, 16, 21]   #to select which object to go to
count=0
placing = [[-0.4, 0.8], [0, 0.4], [0.4, 1.2], [0.8, 0.8]]
suction = 0
print(Robot.end_effector())
time.sleep(2)
print(Robot.end_effector())
for i in  range(7,25): #can use object indices as well (to select particular object)
    object = Robot._objectUids[i]
    pos = p.getBasePositionAndOrientation(object)[0]  
    suction, cons = pick(pos[0], pos[1], object)
    for j in range(181):
        p.resetDebugVisualizerCamera(2, j, -41, [0, -1.4+(2.8*j)/180, 1-j*0.8/180])
        time.sleep(0.01)
    place(x, y, suction, cons)
    for j in range(181):
        p.resetDebugVisualizerCamera(2 , 180-j, -41, [0, 1.4 - (2.8*j)/180, 0.2 + j*0.8/180])
        time.sleep(0.01)
    x = placing[count][0]
    y = placing[count][1]
    count+=1

time.sleep(10)

