from pybullet_envs.bullet.kukaGymEnv import KukaGymEnv
import os
from imageai.Detection.Custom import DetectionModelTrainer,CustomObjectDetection
import cv2
import random
import os
from gym import spaces
import time
import pybullet as p
import kuka
import numpy as np
import pybullet_data
import pdb
import distutils.dir_util
import glob
from pkg_resources import parse_version
import gym
from kuka_discrete_obj_env import KukaDiverseObjectEnv

detector=CustomObjectDetection()
detector.setModelTypeAsYOLOv3()
detector.setModelPath('detection_model-ex-004--loss-0004.076.h5')
detector.setJsonPath('detection_config.json')
detector.loadModel()

env=KukaDiverseObjectEnv()
obs=env.reset()
objects=env._objectUids
#for i in range(len(objects)):
    #print(p.getBasePositionAndOrientation(objects[i])[0][0],p.getBasePositionAndOrientation(objects[i])[0][1])

centroids=[]
def object_detection():
    image=env._get_observation()
    cv2.imwrite("image.jpeg",image)
    detections=detector.detectObjectsFromImage(input_image='image.jpeg',output_image_path='res.jpeg')
    res=cv2.imread('res.jpeg')

    for detection in detections:
        #print(detection["name"], " : ", detection["percentage_probability"], " : ", detection["box_points"])
        coords=detection["box_points"]
        #cv2.rectangle(res,(coords[0],coords[1]),(coords[2],coords[3]),(0, 0, 0) ,2)
        y=(coords[0]+coords[2])/2
        x=(coords[1]+coords[3])/2
        x=0.885-0.0011*x
        y=0.422-0.0012*y
        if detection["name"]=="sphere":
            y=y-0.02
        else:
            y=y-0.02
        #print(x,y)
        centroids.append((x,y))
object_detection()
i=0
x=centroids[i][0]
y=centroids[i][1]
z=0
while True:
    state = p.getLinkState(env._kuka.kukaUid, env._kuka.kukaEndEffectorIndex)
    actualEndEffectorPos = state[0]
    observation,reward,done,_=env.step([(x-actualEndEffectorPos[0])*10,(y-actualEndEffectorPos[1])*10,z,0,0])
    if done==True:
        centroids=[]
        object_detection()
        env._attempted_grasp = False
        env._env_step = 0
        env.terminated = 0
        x=centroids[i][0]
        y=centroids[i][1]

    p.stepSimulation()
    time.sleep(1/100)
