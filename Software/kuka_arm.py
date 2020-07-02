from pybullet_envs.bullet.kukaGymEnv import KukaGymEnv
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
import cv2

env=KukaDiverseObjectEnv()
obs=env.reset()
cubeID=env._objectUids[0]
x=(p.getBasePositionAndOrientation(cubeID)[0])[0]
y=(p.getBasePositionAndOrientation(cubeID)[0])[1]-0.025
z=(p.getBasePositionAndOrientation(cubeID)[0])[2]
cubeornz=(p.getBasePositionAndOrientation(cubeID)[1])[2]
i=0
while True:
    state = p.getLinkState(env._kuka.kukaUid, env._kuka.kukaEndEffectorIndex)
    actualEndEffectorPos = state[0]
    observation,reward,done,_=env.step([(x-actualEndEffectorPos[0])*10,(y-actualEndEffectorPos[1])*10,(z-actualEndEffectorPos[2]),0,0])
    if done==True:
        i+=1
        print("done")
        
        env._attempted_grasp = False
        env._env_step = 0
        env.terminated = 0
        cubeID=env._objectUids[i]
        x=(p.getBasePositionAndOrientation(cubeID)[0])[0]
        y=(p.getBasePositionAndOrientation(cubeID)[0])[1]-0.025
        z=(p.getBasePositionAndOrientation(cubeID)[0])[2]

    p.stepSimulation()
    time.sleep(1/100)
