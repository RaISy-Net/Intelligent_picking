import pybullet as p
import time
import math
import numpy as np
from datetime import datetime
import pybullet_data
from drop_area import *

p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf", [0, 0, 0])
p.setGravity(0, 0, -10)

grip_list= p.loadSDF("./kukka_wsg50/kuka_with_wsg50.sdf")
gripper1 = grip_list[0]


Area_Halfdim = 0.3 # i in real case


'''
x,y,z -> robot base position
scale_ ->scaling params
Inter_area_dist ->distnace between the 2 areas, ideally 20 cm in our case
pickAreaHeight  -> Height of the pick area, ideally 700 ~ 900 cm in our case
'''
MakeArena(x=0,y=0,z=0.05,
	      scale_x=Area_Halfdim,scale_y=Area_Halfdim,scale_z=0,
	      Inter_area_dist=0.5,pickAreaHeight=0.5)





while 1:


  p.resetBasePositionAndOrientation(gripper1,[0,-0.09,0.1],p.getQuaternionFromEuler([0,0,1.57]))
  p.setJointMotorControlArray(gripper1,[0,1,2,3,4,5,6,10],
                              controlMode=p.POSITION_CONTROL,
                              targetPositions=np.zeros(8))
  p.stepSimulation()

  time.sleep(0.01)
p.disconnect()
