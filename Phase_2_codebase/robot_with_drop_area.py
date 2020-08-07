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
    score = gs[0].quality
    angle = gs[0].angle
    print(x,y)
    return x,y,angle,score

def pick(xpos, ypos, object, threshold=0.9):
    Robot.overhead_camera(0)
    Robot.move_frame_and_head(ypos+0.06, xpos-0.03)
    z_init = Robot.end_effector()[0][2]
    x,y,angle,score = get_real_world_coord()
    if score<threshold:
        if z_init>1.185:
            zpos = z_init - 1.181
            Robot.extend_wrist(zpos)
        Robot.move_suction_cup(ypos, xpos)
        Robot.suction_down()
        cons = Robot.suction_force(object)
        Robot.suction_up()
        if z_init>1.185:
            zpos = z_init - 1.181
            Robot.contract_wrist(zpos)
        Robot.reset_gripper()
        return 1, cons
    else:
        Robot.rotate_gripper(angle)
        Robot.move_frame_and_head(y+0.06, x-0.03)
        z_init = Robot.end_effector()[0][2]
        zpos = z_init - 1.052
        Robot.extend_wrist(zpos)
        Robot.close_gripper(0.10)
        Robot.contract_wrist(0.13)
        Robot.reset_gripper()
        return 0, None

def place(xpos, ypos, suction, cons):
    Robot.overhead_camera(1)
    if suction:
        Robot.move_suction_cup(ypos, xpos)
        Robot.extend_arm()
        time.sleep(1)
        Robot.remove_suction_force(cons)
        Robot.contract_arm()
    else:
        Robot.move_frame_and_head(ypos+0.06, xpos-0.03)
        Robot.extend_arm()
        time.sleep(1)
        Robot.open_gripper()
        Robot.contract_arm()
    Robot.reset_gripper()


def grab_drop_suck(xpos, ypos, object):
    Robot.move_frame_and_head(ypos+0.06, xpos-0.03)
    z_init = Robot.end_effector()[0][2]
    x,y,angle,score = get_real_world_coord()
    Robot.rotate_gripper(angle)
    Robot.move_frame_and_head(y+0.06, x-0.03)
    z_init = Robot.end_effector()[0][2]
    zpos = z_init - 1.052
    Robot.extend_wrist(zpos)
    Robot.close_gripper(0.10)
    Robot.contract_wrist(zpos)
    p.setJointMotorControl2(Robot.bot, Robot.plate_left, p.VELOCITY_CONTROL, targetVelocity = 0)
    Robot.open_gripper()
    Robot.reset_gripper()
    pos = p.getBasePositionAndOrientation(object)[0]  
    ypos = pos[1]
    xpos = pos[0]
    Robot.reset_gripper()
    Robot.move_suction_cup(ypos, xpos)
    if z_init>1.185:
            zpos = z_init - 1.183
            Robot.extend_wrist(zpos)
    Robot.suction_down()
    cons = Robot.suction_force(object)
    Robot.suction_up()
    if z_init>1.185:
        zpos = z_init - 1.181
        Robot.contract_wrist(zpos)
    Robot.reset_gripper()
    return 1, cons

Robot = robot()
p.resetDebugVisualizerCamera(2 , 0, -41, [0, -1.4, 1])
Robot.suction_up()
object_indices = [6, 3, 5, 9, 11, 13, 15, 19, 22, 24]   #to select which object to go to
count=0
placing = [[-0.8, 0.4],[-0.8, 0.8], [-0.4, 0.4], [-0.4, 0.8], [0, 0.4], [0, 0.8], [0.4, 0.4], [0.4, 0.8], [0.8, 0.4], [0.8, 0.4]]
suction = 0
time.sleep(5)
print(Robot.end_effector())
for i in  object_indices: #can use object indices as well (to select particular object)
    object = Robot._objectUids[i]
    pos = p.getBasePositionAndOrientation(object)[0]  
    if i==6:
        suction, cons = grab_drop_suck(pos[0], pos[1], object)
    else:
        suction, cons = pick(pos[0], pos[1], object)
    for j in range(181):
        p.resetDebugVisualizerCamera(2, j, -41, [0, -1.4+(2.8*j)/180, 1-j*0.8/180])
        time.sleep(0.01)
    x = placing[count][0]
    y = placing[count][1]
    count+=1
    place(x, y, suction, cons)
    for j in range(181):
        p.resetDebugVisualizerCamera(2 , 180-j, -41, [0, 1.4 - (2.8*j)/180, 0.2 + j*0.8/180])
        time.sleep(0.01)
    
time.sleep(10)

