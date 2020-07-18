from pybullet_envs.bullet.kukaGymEnv import KukaGymEnv
import os
from imageai.Detection.Custom import DetectionModelTrainer,CustomObjectDetection
import cv2
import random
import os
from gym import spaces
import time
import pybullet as p
import kukas
import numpy as np
import pybullet_data
import pdb
import distutils.dir_util
import glob
from pkg_resources import parse_version
import gym
from kuka_discrete_obj_env1 import KukaDiverseObjectEnv
from run_offline import predict_grasp_angle

detector=CustomObjectDetection()
detector.setModelTypeAsYOLOv3()
detector.setModelPath('detection_model-ex-004--loss-0016.868.h5')
detector.setJsonPath('detection_config_object.json')
detector.loadModel()

env=KukaDiverseObjectEnv()
obs=env.reset()
count=0
#for i in range(400):
    #obs=env.reset()
    #objects=env._objectUids
    #for a in range(len(objects)):
        #(x, y, z) = p.getBasePositionAndOrientation(objects[a])[0]
        #env._get_individual_image(x, y, z, count)
        #count+=1
        #time.sleep(2)
    #image=env._get_observation()
    #image=cv2.cvtColor(image,cv2.COLOR_RGB2BGR)
    #cv2.imwrite("C:/Users/yashs/OneDrive/Desktop/PS Simulation/images_new/imagesnew"+str(i)+".jpeg",image)
    #cv2.imwrite("C:/Users/yashs/OneDrive/Desktop/PS Simulation/images_new/depth.jpeg",depth)
centroids=[]
def points():
    objects=env._objectUids
    for i in range(len(objects)):
        print(p.getBasePositionAndOrientation(objects[i])[0][0],p.getBasePositionAndOrientation(objects[i])[0][1],p.getBasePositionAndOrientation(objects[i])[0][2])
        #centroids.append((p.getBasePositionAndOrientation(objects[i])[0][0],p.getBasePositionAndOrientation(objects[i])[0][1]))

def object_detection():
    image=env._get_observation()
    image=cv2.cvtColor(image,cv2.COLOR_RGB2BGR)
    cv2.imwrite("overhead_camera.jpeg",image)

    detections=detector.detectObjectsFromImage(input_image='overhead_camera.jpeg',output_image_path='overhead_res.jpeg')

    for detection in detections:
        print(detection["name"], " : ", detection["percentage_probability"], " : ", detection["box_points"])
        coords=detection["box_points"]
        #cv2.rectangle(res,(coords[0],coords[1]),(coords[2],coords[3]),(0, 0, 0) ,2)
        x=(coords[0]+coords[2])/2
        y=(coords[1]+coords[3])/2
        x=0.233-0.000924*x
        y=-0.742+0.000935*y
        #if detection["name"]=="sphere":
            #y=y+0.0
        #else:
            #y=y+0.05
        #print(x,y)
        centroids.append((x,y))

    gs = get_grasp_prediction(centroids[0][0],centroids[0][1],a)
    x = centroids[0][0]
    y = centroids[0][1]
    y = y-0.0688+((gs[0].center)[0])*0.00026875 - 0.03
    x = x+0.0688-((gs[0].center)[1])*0.00026875 - 0.02
    angle = gs[0].angle
    return x,y,angle

def get_grasp_prediction(x,y,a):
    env._get_individual_image(x,y,0,a)
    network = "trained-models/cornell-randsplit-rgbd-grconvnet3-drop1-ch16/epoch_30_iou_0.97"
    rgb_path = "C:/Users/yashs/OneDrive/Desktop/PS simulation/simulation_images/color"+str(a)+".jpeg"
    depth_path = "C:/Users/yashs/OneDrive/Desktop/PS simulation/simulation_images/depth"+str(a)+".jpeg"
    gs = predict_grasp_angle(network, rgb_path, depth_path)
    return gs
a=0
#points()
x,y,angle = object_detection()
i=0
z=0
print(x,y,angle)
#y=y-0.0688+((gs[0].center)[0])*0.00026875
#x=x+0.0688-((gs[0].center)[1])*0.00026875
#get_grasp_prediction(x,y,0)
while True:
    state = p.getLinkState(env._kuka.kukaUid, env._kuka.kukaEndEffectorIndex)
    actualEndEffectorPos = state[0]
    actualEndEffectorOrn = p.getEulerFromQuaternion(state[1])[2]
    observation,reward,done,_=env.step([(x-actualEndEffectorPos[0])*10,(y-actualEndEffectorPos[1])*7.5,(0.4-actualEndEffectorPos[2]),(angle-actualEndEffectorOrn),[0,0,0,0]])
    if done==True:
        centroids=[]
        x,y,angle = object_detection()
        #print("x and y")
        #print(x,y)
        #print("actual positions")
        #print(actualEndEffectorPos[0],actualEndEffectorPos[1])
        env._attempted_grasp = False
        env._env_step = 0
        env.terminated = 0
        if len(centroids)==0:
            break
        #x=centroids[i][0]
        #y=centroids[i][1]
        print(x,y)
        #print(p.getLinkState(env._kuka.kukaUid, env._kuka.kukaEndEffectorIndex)[0])

    p.stepSimulation()
    #time.sleep(env._timeStep)
    
