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
    network = "trained-models/cornell-randsplit-rgbd-grconvnet3-drop1-ch16/epoch_30_iou_0.97"
    rgb_path = "C:/Users/yashs/OneDrive/Desktop/color"+".png"
    depth_path = "C:/Users/yashs/OneDrive/Desktop/depth"+".png"
    gs = predict_grasp_angle(network, rgb_path, depth_path)
    return gs

def get_real_world_coord():
    end_effector_initpos = Robot.end_effector()[0]
    a=0
    gs = get_grasp_prediction(end_effector_initpos[0],end_effector_initpos[1],end_effector_initpos[2],a)
    x = end_effector_initpos[0]
    y = end_effector_initpos[1]
    z = end_effector_initpos[2]
    a = 0.4896 - 0.4663*z
    y = y - a*(1-gs[0].center[0]/112)
    x = x + a*(1-gs[0].center[1]/112)
    angle = gs[0].angle
    print(x,y)
    return x,y,angle

def pick(xpos, ypos):
    Robot.suction_up()
    Robot.move_frame(ypos+0.06)
    Robot.move_head(xpos-0.03)
    x,y,angle = get_real_world_coord()
    Robot.extend_wrist(0.05)
    Robot.rotate_gripper(angle)
    Robot.extend_wrist(0.08)
    Robot.close_gripper(0.09)
    Robot.contract_wrist(0.13)

def place(xpos, ypos):
    Robot.move_frame(ypos+0.06)
    Robot.move_head(xpos-0.03)
    Robot.extend_arm()
    Robot.extend_wrist(0.05)
    Robot.open_gripper()
    Robot.contract_arm()

Robot = robot()
end_effector_initpos = Robot.end_effector()[0]
print(end_effector_initpos)
object = Robot._objectUids[11]
pos = p.getBasePositionAndOrientation(object)[0]
pick(pos[0], pos[1])
place(0.8, 0.8)

time.sleep(10)

