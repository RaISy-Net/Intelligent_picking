import pybullet as p
import time
import math
import numpy as np
import random
from datetime import datetime
import pybullet_data
from drop_area import *
from robot_frame_mass import *
from bot_maker import *

shift = [0, -0.02, 0]

p.connect(p.GUI)
urdfRoot=pybullet_data.getDataPath()
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf", [0, 0, 0])
p.setGravity(0, 0, -10)
sphere = p.loadURDF('sphere_small.urdf',[0.6,-1.5,1.2])

for i in range(5):
	urdf_no = str(random.randint(500,999))
	p.loadURDF('random_urdfs/'+urdf_no+'/'+urdf_no+'.urdf',[-0.8+0.4*i,2.1,0.5])

for i in range(12):
	urdf_no = str(random.randint(100,500))
	p.loadURDF('random_urdfs/'+urdf_no+'/'+urdf_no+'.urdf',[-0.8+0.5*(i%4),-0.5-0.3*(i%3),1.2])
for i in range(6):
	urdf_no = str(random.randint(500,999))
	p.loadURDF('random_urdfs/'+urdf_no+'/'+urdf_no+'.urdf',[-0.8+0.5*(i%2),-1.5-0.3*(i%3),1.2])
#to make the things fall down





extra = []

# grip_list= p.loadSDF("./kukka_wsg50/kuka_with_wsg50.sdf")
# gripper1 = grip_list[0]


Area_Halfdim = 1 # i in real case
p.resetDebugVisualizerCamera(4, 45, -45, [0,0,0])

'''
x,y,z -> robot base position
scale_ ->scaling params
Inter_area_dist ->distnace between the 2 areas, ideally 20 cm in our case
pickAreaHeight  -> Height of the pick area, ideally 700 ~ 900 cm in our case
'''
MakeArena(x=0,y=0,z=0.05,
	      scale_x=Area_Halfdim,scale_y=Area_Halfdim,scale_z=0,
	      Inter_area_dist=0.5,pickAreaHeight=0.9)

base1, base2 ,link1,link2,l2= MakeRobot(x=0,y=0,z=0.05,
	      scale_x=Area_Halfdim,scale_y=Area_Halfdim,scale_z=0,
	      Inter_area_dist=0.2,pickAreaHeight=0.9)

# MakeBot(x=0,y=0,z=height,l1=0.2,l2=0.2)

# print(height,"--------------")

gripper=p.loadSDF('gripper/wsg50_one_motor_gripper.sdf')
gripper=gripper[0]

x=0
i = 0
'''
while(i<2000):
	p.stepSimulation()
	time.sleep(0.01)
	i += 1
'''
for i in range(500):
	p.stepSimulation()
#logg = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4,'filename2.mp4')
'''
while 1:
	# x=x+0.01
	# base1, base2 , height= MakeRobot(x=0,y=x,z=0.05,
	#       scale_x=Area_Halfdim,scale_y=Area_Halfdim,scale_z=0,
	#       Inter_area_dist=0.2,pickAreaHeight=0.9)

	# MakeBot(x=0,y=x,z=height,l1=0.2,l2=0.2)
	link2_pos,link2_orn=p.getBasePositionAndOrientation(link2)
	while(link2_pos[1]>-1.5):
		p.resetBasePositionAndOrientation(gripper,[link2_pos[0],link2_pos[1],link2_pos[2]-l2],p.getQuaternionFromEuler([0,np.pi,np.pi/2]))
		p.applyExternalForce(base1,-1,[0,-20,0],[0,0,0],p.LINK_FRAME)
		p.applyExternalForce(base2,-1,[0,-20,0],[0,0,0],p.LINK_FRAME)
		# p.applyExternalForce(link2,-1,[0.001,0,0],[0,0,0],p.LINK_FRAME)
		link2_pos,link2_orn=p.getBasePositionAndOrientation(link2)
		p.stepSimulation()
		time.sleep(0.01)
	p.resetBaseVelocity(base1,[0,0,0])
	p.resetBaseVelocity(base2,[0,0,0])
	p.resetBaseVelocity(link1,[0,0,0])
	p.resetBaseVelocity(link2,[0,0,0])
	link2_pos,link2_orn=p.getBasePositionAndOrientation(link2)
	p.resetBasePositionAndOrientation(gripper,[link2_pos[0],link2_pos[1],link2_pos[2]-l2],p.getQuaternionFromEuler([0,np.pi,np.pi/2]))
	p.stepSimulation()
	time.sleep(0.01)
	i = 0

	while(i<100):
		p.resetBaseVelocity(base1,[0,0,0])
		p.resetBaseVelocity(base2,[0,0,0])
		p.resetBaseVelocity(link1,[0,0,0])
		p.resetBaseVelocity(link2,[0,0,0])
		link2_pos,link2_orn=p.getBasePositionAndOrientation(link2)
		p.resetBasePositionAndOrientation(gripper,[link2_pos[0],link2_pos[1],link2_pos[2]-l2],p.getQuaternionFromEuler([0,np.pi,np.pi/2]))

		p.stepSimulation()
		time.sleep(0.01)
		i += 1
	while(link2_pos[0]<0.6):
		p.resetBasePositionAndOrientation(gripper,[link2_pos[0],link2_pos[1],link2_pos[2]-l2],p.getQuaternionFromEuler([0,np.pi,np.pi/2]))
		p.applyExternalForce(link2,-1,[0.001,0,0],[0,0,0],p.LINK_FRAME)
		link2_pos,link2_orn=p.getBasePositionAndOrientation(link2)
		p.stepSimulation()
		time.sleep(0.01)
	p.resetBaseVelocity(base1,[0,0,0])
	p.resetBaseVelocity(base2,[0,0,0])
	p.resetBaseVelocity(link1,[0,0,0])
	p.resetBaseVelocity(link2,[0,0,0])
	link2_pos,link2_orn=p.getBasePositionAndOrientation(link2)
	p.resetBasePositionAndOrientation(gripper,[link2_pos[0],link2_pos[1],link2_pos[2]-l2],p.getQuaternionFromEuler([0,np.pi,np.pi/2]))
	while(i<100):
		p.resetBaseVelocity(base1,[0,0,0])
		p.resetBaseVelocity(base2,[0,0,0])
		p.resetBaseVelocity(link1,[0,0,0])
		p.resetBaseVelocity(link2,[0,0,0])
		link2_pos,link2_orn=p.getBasePositionAndOrientation(link2)
		p.resetBasePositionAndOrientation(gripper,[link2_pos[0],link2_pos[1],link2_pos[2]-l2],p.getQuaternionFromEuler([0,np.pi,np.pi/2]))

		p.stepSimulation()
		time.sleep(0.01)
		i += 1
	sphere_pos,sphere_orn=p.getBasePositionAndOrientation(sphere)
	gripper_pos,gripper_orn=p.getBasePositionAndOrientation(gripper)
	i = 0.001
	scale_x =1
	j = 0
	extra_c = p.createCollisionShape(p.GEOM_BOX,halfExtents=[scale_x/20,scale_x/20,i/2])
	extra_v = p.createVisualShape(p.GEOM_BOX,halfExtents=[scale_x/20,scale_x/20,i/2],rgbaColor=[0.952,0.447,0.125,1])
	extra.append(p.createMultiBody(baseCollisionShapeIndex=extra_c,baseVisualShapeIndex=extra_v,basePosition=[link2_pos[0],link2_pos[1],link2_pos[2]-l2-i/2]))
	while(gripper_pos[2]>sphere_pos[2]+0.2):
		scale_x = 1
		p.resetBasePositionAndOrientation(gripper,[link2_pos[0],link2_pos[1],link2_pos[2]-l2-i],p.getQuaternionFromEuler([0,np.pi,np.pi/2]))
		extra_c = p.createCollisionShape(p.GEOM_BOX,halfExtents=[scale_x/20,scale_x/20,i/2])
		extra_v = p.createVisualShape(p.GEOM_BOX,halfExtents=[scale_x/20,scale_x/20,i/2],rgbaColor=[0.952,0.447,0.125,1])
		j = j + 1
		extra.append(p.createMultiBody(baseCollisionShapeIndex=extra_c,baseVisualShapeIndex=extra_v,basePosition=[link2_pos[0],link2_pos[1],link2_pos[2]-l2-i/2]))
		sphere_pos,sphere_orn=p.getBasePositionAndOrientation(sphere)
		gripper_pos,gripper_orn=p.getBasePositionAndOrientation(gripper)
		p.resetBasePositionAndOrientation(extra[j-1],[100,100,100],p.getQuaternionFromEuler([0,0,0]))
		i = i + 0.01
		p.stepSimulation()
		time.sleep(0.01)
	p.resetBasePositionAndOrientation(extra[j],[100,100,100],p.getQuaternionFromEuler([0,0,0]))
	p.resetBasePositionAndOrientation(sphere,[gripper_pos[0],gripper_pos[1],gripper_pos[2]-0.15],p.getQuaternionFromEuler([0,0,0]))
	gripper_pos,gripper_orn=p.getBasePositionAndOrientation(gripper)
	extra = []
	j = 0
	extra_c = p.createCollisionShape(p.GEOM_BOX,halfExtents=[scale_x/20,scale_x/20,i/2])
	extra_v = p.createVisualShape(p.GEOM_BOX,halfExtents=[scale_x/20,scale_x/20,i/2],rgbaColor=[0.952,0.447,0.125,1])
	extra.append(p.createMultiBody(baseCollisionShapeIndex=extra_c,baseVisualShapeIndex=extra_v,basePosition=[link2_pos[0],link2_pos[1],link2_pos[2]-l2-i/2]))
	while(gripper_pos[2]<link2_pos[2]-l2):
		p.resetBasePositionAndOrientation(gripper,[link2_pos[0],link2_pos[1],link2_pos[2]-l2-i],p.getQuaternionFromEuler([0,np.pi,np.pi/2]))
		p.resetBasePositionAndOrientation(sphere,[gripper_pos[0],gripper_pos[1],gripper_pos[2]-0.15],p.getQuaternionFromEuler([0,0,0]))
		extra_c = p.createCollisionShape(p.GEOM_BOX,halfExtents=[scale_x/20,scale_x/20,i/2])
		extra_v = p.createVisualShape(p.GEOM_BOX,halfExtents=[scale_x/20,scale_x/20,i/2],rgbaColor=[0.952,0.447,0.125,1])
		extra.append(p.createMultiBody(baseCollisionShapeIndex=extra_c,baseVisualShapeIndex=extra_v,basePosition=[link2_pos[0],link2_pos[1],link2_pos[2]-l2-i/2]))
		gripper_pos,gripper_orn=p.getBasePositionAndOrientation(gripper)
		p.resetBasePositionAndOrientation(extra[j-1],[100,100,100],p.getQuaternionFromEuler([0,0,0]))
		i = i - 0.01
		p.stepSimulation()
		time.sleep(0.01)
	p.resetBasePositionAndOrientation(extra[j],[100,100,100],p.getQuaternionFromEuler([0,0,0]))
	link2_pos,link2_orn=p.getBasePositionAndOrientation(link2)
	while(link2_pos[0]>0):
		p.resetBasePositionAndOrientation(gripper,[link2_pos[0],link2_pos[1],link2_pos[2]-l2],p.getQuaternionFromEuler([0,np.pi,np.pi/2]))
		p.applyExternalForce(link2,-1,[-0.001,0,0],[0,0,0],p.LINK_FRAME)
		link2_pos,link2_orn=p.getBasePositionAndOrientation(link2)
		p.resetBasePositionAndOrientation(gripper,[link2_pos[0],link2_pos[1],link2_pos[2]-l2],p.getQuaternionFromEuler([0,np.pi,np.pi/2]))
		gripper_pos,gripper_orn=p.getBasePositionAndOrientation(gripper)
		p.resetBasePositionAndOrientation(sphere,[gripper_pos[0],gripper_pos[1],gripper_pos[2]-0.15],p.getQuaternionFromEuler([0,0,0]))
		p.stepSimulation()
		time.sleep(0.01)
	p.resetBaseVelocity(base1,[0,0,0])
	p.resetBaseVelocity(base2,[0,0,0])
	p.resetBaseVelocity(link1,[0,0,0])
	p.resetBaseVelocity(link2,[0,0,0])
	link2_pos,link2_orn=p.getBasePositionAndOrientation(link2)
	p.resetBasePositionAndOrientation(gripper,[link2_pos[0],link2_pos[1],link2_pos[2]-l2],p.getQuaternionFromEuler([0,np.pi,np.pi/2]))
	gripper_pos,gripper_orn=p.getBasePositionAndOrientation(gripper)
	p.resetBasePositionAndOrientation(sphere,[gripper_pos[0],gripper_pos[1],gripper_pos[2]-0.15],p.getQuaternionFromEuler([0,0,0]))
	i=0
	while(i<100):
		p.resetBaseVelocity(base1,[0,0,0])
		p.resetBaseVelocity(base2,[0,0,0])
		p.resetBaseVelocity(link1,[0,0,0])
		p.resetBaseVelocity(link2,[0,0,0])
		link2_pos,link2_orn=p.getBasePositionAndOrientation(link2)
		p.resetBasePositionAndOrientation(gripper,[link2_pos[0],link2_pos[1],link2_pos[2]-l2],p.getQuaternionFromEuler([0,np.pi,np.pi/2]))
		gripper_pos,gripper_orn=p.getBasePositionAndOrientation(gripper)
		p.resetBasePositionAndOrientation(sphere,[gripper_pos[0],gripper_pos[1],gripper_pos[2]-0.15],p.getQuaternionFromEuler([0,0,0]))

		p.stepSimulation()
		time.sleep(0.01)
		i += 1	
	while(link2_pos[1]<0):
		p.resetBasePositionAndOrientation(gripper,[link2_pos[0],link2_pos[1],link2_pos[2]-l2],p.getQuaternionFromEuler([0,np.pi,np.pi/2]))
		gripper_pos,gripper_orn=p.getBasePositionAndOrientation(gripper)
		p.resetBasePositionAndOrientation(sphere,[gripper_pos[0],gripper_pos[1],gripper_pos[2]-0.15],p.getQuaternionFromEuler([0,0,0]))
		p.applyExternalForce(base1,-1,[0,20,0],[0,0,0],p.LINK_FRAME)
		p.applyExternalForce(base2,-1,[0,20,0],[0,0,0],p.LINK_FRAME)
		# p.applyExternalForce(link2,-1,[0.001,0,0],[0,0,0],p.LINK_FRAME)
		link2_pos,link2_orn=p.getBasePositionAndOrientation(link2)
		p.stepSimulation()
		time.sleep(0.01)
	p.resetBaseVelocity(base1,[0,0,0])
	p.resetBaseVelocity(base2,[0,0,0])
	p.resetBaseVelocity(link1,[0,0,0])
	p.resetBaseVelocity(link2,[0,0,0])
	link2_pos,link2_orn=p.getBasePositionAndOrientation(link2)
	p.resetBasePositionAndOrientation(gripper,[link2_pos[0],link2_pos[1],link2_pos[2]-l2],p.getQuaternionFromEuler([0,np.pi,np.pi/2]))
	gripper_pos,gripper_orn=p.getBasePositionAndOrientation(gripper)
	p.resetBasePositionAndOrientation(sphere,[gripper_pos[0],gripper_pos[1],gripper_pos[2]-0.15],p.getQuaternionFromEuler([0,0,0]))
	p.stepSimulation()
	time.sleep(0.01)
	i = 0

	while(i<100):
		p.resetBaseVelocity(base1,[0,0,0])
		p.resetBaseVelocity(base2,[0,0,0])
		p.resetBaseVelocity(link1,[0,0,0])
		p.resetBaseVelocity(link2,[0,0,0])
		link2_pos,link2_orn=p.getBasePositionAndOrientation(link2)
		p.resetBasePositionAndOrientation(gripper,[link2_pos[0],link2_pos[1],link2_pos[2]-l2],p.getQuaternionFromEuler([0,np.pi,np.pi/2]))
		gripper_pos,gripper_orn=p.getBasePositionAndOrientation(gripper)
		p.resetBasePositionAndOrientation(sphere,[gripper_pos[0],gripper_pos[1],gripper_pos[2]-0.15],p.getQuaternionFromEuler([0,0,0]))

		p.stepSimulation()
		time.sleep(0.01)
		i += 1		
			
	#p.stopStateLogging(logg)
	while(1):
		pass	
	

	
p.disconnect()
'''