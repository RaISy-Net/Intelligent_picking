import pybullet as p
import time
import math
import numpy as np
from datetime import datetime
import pybullet_data
from drop_area import *
from robot_frame_mass import *
from bot_maker import *

p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf", [0, 0, 0])
p.setGravity(0, 0, -10)

# grip_list= p.loadSDF("./kukka_wsg50/kuka_with_wsg50.sdf")
# gripper1 = grip_list[0]


gripper=p.loadSDF('gripper/wsg50_one_motor_gripper.sdf')
gripper=gripper[0]
	# print(p.getLinkSta
while 1:
	# x=x+0.01
	# base1, base2 , height= MakeRobot(x=0,y=x,z=0.05,
	#       scale_x=Area_Halfdim,scale_y=Area_Halfdim,scale_z=0,
	#       Inter_area_dist=0.2,pickAreaHeight=0.9)

	# MakeBot(x=0,y=x,z=height,l1=0.2,l2=0.2)

	# p.applyExternalForce(base1,-1,[0,20,0],[0,0,0],p.LINK_FRAME)
	# p.applyExternalForce(base2,-1,[0,20,0],[0,0,0],p.LINK_FRAME)
	p.stepSimulation()
	time.sleep(0.01)
p.disconnect()
