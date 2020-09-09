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
from grasp_estimation import GraspEstimation

Area_Halfdim=1

class GridEnvironment():
    def __init__(self):
        #loading the robot class
        self.Robot = robot()
        self.Robot.suction_up()
        suction = 0

        #reseting the camera
        p.resetDebugVisualizerCamera(2 , 0, -41, [0, -1.4, 1])
        
        #setting objects to be picked and where to be dropped
        self.object_indices = [2, 7, 8, 6, 24]  
        self.count=0
        self.placing = [[0, 0.8], [0.8, 0.4], [0.8, 0.8]]

        #initializing the grasp model
        self.GraspModel = GraspEstimation()

    #function to get RGB-D image and predict the grasping points info
    def get_grasp_prediction(self,x,y,z,a):
        self.Robot.rgbd_images(x,y,z)
        self.network = "./trained models/epoch_17_iou_0.96"
        self.rgb_path = "./CapturedImg/color"+".png"
        self.depth_path = "./CapturedImg/depth"+".png"
        self.GraspModel.load_network_image(self.network, self.rgb_path, self.depth_path)
        gs = self.GraspModel.predict_grasp()
        return gs

    #function to extract points from the info received from the function above
    def get_real_world_coord(self):
        end_effector_initpos = self.Robot.end_effector()[0]
        a=0
        gs = self.get_grasp_prediction(end_effector_initpos[0],end_effector_initpos[1],end_effector_initpos[2],a)
        x = end_effector_initpos[0]
        y = end_effector_initpos[1]
        z = end_effector_initpos[2]
        a = 0.005
        y = y - a*(1-gs[0].center[0]/112)
        x = x + a*(1-gs[0].center[1]/112)
        score = gs[0].quality
        angle = gs[0].angle
        return x,y,angle,score

    #function to pick an object autonomously
    def pick(self, xpos, ypos, object, threshold=0.9):
        self.Robot.overhead_camera(0)
        self.Robot.move_frame_and_head(ypos+0.06, xpos-0.03)
        z_init = self.Robot.end_effector()[0][2]
        x,y,angle,score = self.get_real_world_coord()
        if score<threshold:
            if z_init>1.185:
                zpos = z_init - 1.181
                self.Robot.extend_wrist(zpos)
            self.Robot.move_suction_cup(ypos, xpos)
            self.Robot.close_gripper(0.09, 1)
            p.resetDebugVisualizerCamera(0.4, angle, -20, [xpos, ypos, p.getBasePositionAndOrientation(object)[0][2]])
            self.Robot.suction_down()
            cons = self.Robot.suction_force(object)
            self.Robot.suction_up()
            p.resetDebugVisualizerCamera(2 , 0, -41, [0, -1.4, 1])
            if z_init>1.185:
                zpos = z_init - 1.181
                self.Robot.contract_wrist(zpos)
                self.Robot.reset_gripper()
                return 1, cons
        else:
            self.Robot.rotate_gripper(angle)
            self.Robot.move_frame_and_head(y+0.06, x-0.03)
            z_init = self.Robot.end_effector()[0][2]
            zpos = z_init - 1.052
            p.resetDebugVisualizerCamera(0.4, angle, -20, [xpos, ypos, p.getBasePositionAndOrientation(object)[0][2]])
            self.Robot.extend_wrist(zpos)
            self.Robot.close_gripper(0.10)
            self.Robot.contract_wrist(0.13)
            p.resetDebugVisualizerCamera(2 , 0, -41, [0, -1.4, 1])
            self.Robot.reset_gripper()
            return 0, None

    #function to place objects
    def place(self, xpos, ypos, suction, cons):
        self.Robot.overhead_camera(1)
        if suction:
            self.Robot.move_suction_cup(ypos, xpos)
            self.Robot.extend_arm(xpos, ypos)
            self.Robot.remove_suction_force(cons)
            self.Robot.contract_arm()
            self.Robot.open_gripper()
        else:
            self.Robot.move_frame_and_head(ypos+0.06, xpos-0.03)
            self.Robot.extend_arm(xpos, ypos)
            self.Robot.open_gripper()
            self.Robot.contract_arm()
        self.Robot.reset_gripper()

    #function to grab drop and suck
    def grab_drop_suck(self, xpos, ypos, object):
        self.Robot.move_frame_and_head(ypos+0.06, xpos-0.03)
        z_init = self.Robot.end_effector()[0][2]
        x,y,angle,score = self.get_real_world_coord()
        self.Robot.rotate_gripper(angle)
        self.Robot.move_frame_and_head(y+0.06, x-0.03)
        z_init = self.Robot.end_effector()[0][2]
        zpos = z_init - 1.052
        p.resetDebugVisualizerCamera(0.4, angle, -20, [xpos, ypos, p.getBasePositionAndOrientation(object)[0][2]])
        self.Robot.extend_wrist(zpos)
        self.Robot.close_gripper(0.10)
        self.Robot.contract_wrist(zpos)
        p.setJointMotorControl2(Robot.bot, Robot.plate_left, p.VELOCITY_CONTROL, targetVelocity = 0)
        self.Robot.open_gripper()
        self.Robot.reset_gripper()
        pos = p.getBasePositionAndOrientation(object)[0]  
        ypos = pos[1]
        xpos = pos[0]
        self.Robot.reset_gripper()
        self.Robot.move_suction_cup(ypos, xpos)
        if z_init>1.185:
                zpos = z_init - 1.183
                self.Robot.extend_wrist(zpos)
        self.Robot.close_gripper(0.09, 1)
        self.Robot.suction_down()
        cons = self.Robot.suction_force(object)
        self.Robot.suction_up()
        p.resetDebugVisualizerCamera(2 , 0, -41, [0, -1.4, 1])
        if z_init>1.185:
            zpos = z_init - 1.181
            self.Robot.contract_wrist(zpos)
        self.Robot.reset_gripper()
        return 1, cons

    #single function to run tnhe entire simulation
    def do_simulation(self):
        for i in  self.object_indices: 
            object = self.Robot._objectUids[i]
            pos = p.getBasePositionAndOrientation(object)[0]  
            if i==6:
                suction, cons = self.grab_drop_suck(pos[0], pos[1], object)
            else:
                suction, cons = self.pick(pos[0], pos[1], object)
            for j in range(181):
                p.resetDebugVisualizerCamera(2, j, -41, [0, -1.4+(2.8*j)/180, 1-j*0.8/180])
                time.sleep(0.01)
            x = self.placing[self.count][0]
            y = self.placing[self.count][1]
            self.count+=1
            self.place(x, y, suction, cons)
            for j in range(181):
                p.resetDebugVisualizerCamera(2 , 180-j, -41, [0, 1.4 - (2.8*j)/180, 0.2 + j*0.8/180])
                time.sleep(0.01)