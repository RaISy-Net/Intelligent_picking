import pybullet as p
import time
import math
import numpy as np
from datetime import datetime
import pybullet_data


def MakeRobot(x,y,z=0.5,scale_x=1,scale_y=1,scale_z=0.5,Inter_area_dist=1,pickAreaHeight=0.9):
	#p.resetDebugVisualizerCamera(2, 0, -89, [0,0,2])
	p.setAdditionalSearchPath(pybullet_data.getDataPath())
	strip_c = p.createCollisionShape(p.GEOM_BOX,halfExtents=[scale_x/20,(scale_y+(Inter_area_dist/2))*2,0.01])
	strip_v = p.createVisualShape(p.GEOM_BOX,halfExtents=[scale_x/20,(scale_y+(Inter_area_dist/2))*2,0.01],rgbaColor=[0,0,0,1])
	y_pick = y - scale_y - Inter_area_dist/2.0
	strip1 = p.createMultiBody(baseMass = 10,baseCollisionShapeIndex=strip_c,baseVisualShapeIndex=strip_v,basePosition=[scale_x + 0.1,y ,z+scale_z])
	strip2 = p.createMultiBody(baseMass = 10,baseCollisionShapeIndex=strip_c,baseVisualShapeIndex=strip_v,basePosition=[-scale_x - 0.1,y ,z+scale_z])
	base_c = p.createCollisionShape(p.GEOM_BOX,halfExtents=[scale_x/20,scale_x/20,0.07])
	base_v = p.createVisualShape(p.GEOM_BOX,halfExtents=[scale_x/20,scale_x/20,0.07],rgbaColor=[1,0.51,0,1])
	base1 = p.createMultiBody(baseMass = 5,baseCollisionShapeIndex=base_c,baseVisualShapeIndex=base_v,basePosition=[scale_x + 0.1,y ,z+scale_z+0.075])
	base2 = p.createMultiBody(baseMass = 5,baseCollisionShapeIndex=base_c,baseVisualShapeIndex=base_v,basePosition=[-scale_x - 0.1,y ,z+scale_z+0.075])
	pole_c = p.createCollisionShape(p.GEOM_CYLINDER,height = 0.6+pickAreaHeight, radius = scale_x/45)
	pole_v = p.createVisualShape(p.GEOM_CYLINDER,length = 0.6+pickAreaHeight, radius = scale_x/45, rgbaColor=[0.4,1,0,1])
	pole1 = p.createMultiBody(baseMass = 0.1,baseCollisionShapeIndex=pole_c,baseVisualShapeIndex=pole_v,basePosition=[scale_x + 0.1,y ,z+scale_z+0.15+(0.6+pickAreaHeight)/2])
	pole2 = p.createMultiBody(baseMass = 0.1,baseCollisionShapeIndex=pole_c,baseVisualShapeIndex=pole_v,basePosition=[-scale_x - 0.1,y ,z+scale_z+0.15+(0.6+pickAreaHeight)/2])
	upper_base_c = p.createCollisionShape(p.GEOM_BOX,halfExtents=[scale_x/40,scale_x/20,0.02])
	upper_base_v = p.createVisualShape(p.GEOM_BOX,halfExtents=[scale_x/40,scale_x/20,0.02], rgbaColor = [0,0.39,0,1])
	upper_base1 = p.createMultiBody(baseMass = 0.01,baseCollisionShapeIndex=upper_base_c,baseVisualShapeIndex=upper_base_v,basePosition=[scale_x + 0.1,y ,z+scale_z+0.15+0.6+pickAreaHeight+0.02])
	upper_base2 = p.createMultiBody(baseMass = 0.01,baseCollisionShapeIndex=upper_base_c,baseVisualShapeIndex=upper_base_v,basePosition=[-scale_x - 0.1,y ,z+scale_z+0.15+0.6+pickAreaHeight+0.02])
	h_pole_c = p.createCollisionShape(p.GEOM_BOX,halfExtents=[(scale_x+0.1-scale_x/40),scale_x/80,0.02])
	h_pole_v = p.createVisualShape(p.GEOM_BOX,halfExtents=[(scale_x+0.1-scale_x/40),scale_x/80,0.02], rgbaColor=[0,0.39,0,1])
	h_pole1 = p.createMultiBody(baseMass = 0.0001,baseCollisionShapeIndex=h_pole_c,baseVisualShapeIndex=h_pole_v,basePosition=[x,y + scale_x/40 ,z+scale_z+0.15+0.6+pickAreaHeight+0.02])
	h_pole2 = p.createMultiBody(baseMass = 0.0001,baseCollisionShapeIndex=h_pole_c,baseVisualShapeIndex=h_pole_v,basePosition=[x,y - scale_x/40,z+scale_z+0.15+0.6+pickAreaHeight+0.02])
	p.createConstraint(base1, -1, pole1, -1, p.JOINT_FIXED, [0,0,1], [0, 0, 0.07], [0,0,-(0.6+pickAreaHeight)/2])
	p.createConstraint(base2, -1, pole2, -1, p.JOINT_FIXED, [0,0,1], [0, 0, 0.07], [0,0,-(0.6+pickAreaHeight)/2])
	p.createConstraint(pole1, -1, upper_base1, -1, p.JOINT_FIXED, [0,0,1], [0, 0, (0.6+pickAreaHeight)/2], [0,0,-0.02])
	p.createConstraint(pole2, -1, upper_base2, -1, p.JOINT_FIXED, [0,0,1], [0, 0, (0.6+pickAreaHeight)/2], [0,0,-0.02])
	p.createConstraint(upper_base1, -1, h_pole1, -1, p.JOINT_FIXED, [1,0,0], [-scale_x/40, scale_x/40, 0], [(scale_x+0.1-scale_x/40),0,0])
	p.createConstraint(upper_base1, -1, h_pole2, -1, p.JOINT_FIXED, [1,0,0], [-scale_x/40, -scale_x/40, 0], [(scale_x+0.1-scale_x/40),0,0])
	p.createConstraint(upper_base2, -1, h_pole1, -1, p.JOINT_FIXED, [1,0,0], [scale_x/40, scale_x/40, 0], [-(scale_x+0.1-scale_x/40),0,0])
	p.createConstraint(upper_base2, -1, h_pole2, -1, p.JOINT_FIXED, [1,0,0], [scale_x/40, -scale_x/40, 0], [-(scale_x+0.1-scale_x/40),0,0])
	p.createConstraint(strip1, -1, -1, -1, p.JOINT_FIXED, [0,0,0], [0,0, 0], [scale_x + 0.1,y ,z+scale_z])
	p.createConstraint(strip2, -1, -1, -1, p.JOINT_FIXED, [0,0,0], [0,0, 0], [-scale_x - 0.1,y ,z+scale_z])
	# p.createConstraint(base1, -1, strip1, -1, p.JOINT_PRISMATIC, [0,1,0], [0,0, -0.075], [0,0,0])
	# p.createConstraint(base2, -1, strip2, -1, p.JOINT_PRISMATIC, [0,1,0], [0,0, -0.075], [0,0,0])



	z=z+scale_z+0.15+0.6+pickAreaHeight-0.08
	l1=0.2
	l2=0.2
	head_x=0.15
	head_y=0.15
	head_z=0.05
	head_c=p.createCollisionShape(p.GEOM_BOX,halfExtents=[head_x,head_y,head_z])
	head_v=p.createVisualShape(p.GEOM_BOX,halfExtents=[head_x,head_y,head_z],rgbaColor=[1,0,0,1])
	link1_r=0.07
	link1_h=l1
	link1_c=p.createCollisionShape(p.GEOM_CYLINDER,radius=link1_r,height=link1_h)
	link1_v=p.createVisualShape(p.GEOM_CYLINDER,radius=link1_r,length=link1_h,rgbaColor=[1,0,1,1])
	link2_r=0.05
	link2_h=l2
	link2_c=p.createCollisionShape(p.GEOM_CYLINDER,radius=link2_r,height=link2_h)
	link2_v=p.createVisualShape(p.GEOM_CYLINDER,radius=link2_r,length=link2_h,rgbaColor=[1,1,1,1])
	servo1_x=0.05
	servo1_y=0.03
	servo1_z=0.02
	servo1_c=p.createCollisionShape(p.GEOM_BOX,halfExtents=[servo1_x,servo1_y,servo1_z])
	servo1_v=p.createVisualShape(p.GEOM_BOX,halfExtents=[servo1_x,servo1_y,servo1_z],rgbaColor=[0,0,0,1])

	d_bracket1_x=0.002
	d_bracket1_y=0.002
	d_bracket1_z=0.002

	d_bracket1_c=p.createCollisionShape(p.GEOM_MESH,fileName='d_bracket.stl',meshScale=[d_bracket1_x,d_bracket1_y,d_bracket1_z])
	d_bracket1_v=p.createVisualShape(p.GEOM_MESH,fileName='d_bracket.stl',meshScale=[d_bracket1_x,d_bracket1_y,d_bracket1_z],rgbaColor=[0,0,0,1])

	servo2_x=0.03
	servo2_y=0.05
	servo2_z=0.02
	servo2_c=p.createCollisionShape(p.GEOM_BOX,halfExtents=[servo2_x,servo2_y,servo2_z])
	servo2_v=p.createVisualShape(p.GEOM_BOX,halfExtents=[servo2_x,servo2_y,servo2_z],rgbaColor=[0,0,0,1])

	d_bracket2_x=0.002
	d_bracket2_y=0.002
	d_bracket2_z=0.002

	d_bracket2_c=p.createCollisionShape(p.GEOM_MESH,fileName='d_bracket2.stl',meshScale=[d_bracket2_x,d_bracket2_y,d_bracket2_z])
	d_bracket2_v=p.createVisualShape(p.GEOM_MESH,fileName='d_bracket2.stl',meshScale=[d_bracket2_x,d_bracket2_y,d_bracket2_z],rgbaColor=[0,0,0,1])
	


	multi_head=p.createMultiBody(baseMass=0.0001,baseCollisionShapeIndex=head_c,baseVisualShapeIndex=head_v,basePosition=[x,y,z])
	multi_link1=p.createMultiBody(baseMass=0.0001,baseCollisionShapeIndex=link1_c,baseVisualShapeIndex=link1_v,basePosition=[x,y,z-link1_h/2.0-head_z])
	multi_link2=p.createMultiBody(baseMass=0.0001,baseCollisionShapeIndex=link2_c,baseVisualShapeIndex=link2_v,basePosition=[x,y,z-link2_h/2.0-head_z-link1_h])
	multi_servo1=p.createMultiBody(baseMass=0.0001,baseCollisionShapeIndex=servo1_c,baseVisualShapeIndex=servo1_v,basePosition=[x,y,z-link1_h-link2_h-head_z-servo1_z])
	# multi_d_bracket1=p.createMultiBody(baseMass=0.0001,baseCollisionShapeIndex=d_bracket1_c,baseVisualShapeIndex=d_bracket1_v,basePosition=[x-0.01,y,z-link1_h-link2_h-head_z-2.5*servo1_z])
	multi_servo2=p.createMultiBody(baseMass=0.0001,baseCollisionShapeIndex=servo2_c,baseVisualShapeIndex=servo2_v,basePosition=[x,y,z-link1_h-link2_h-head_z-2.5*servo1_z-servo2_z])
	# multi_d_bracket2=p.createMultiBody(baseMass=0.0001,baseCollisionShapeIndex=d_bracket2_c,baseVisualShapeIndex=d_bracket2_v,basePosition=[x-0.01,y,z-link1_h-link2_h-head_z-2.5*servo1_z-2.5*servo2_z])

	gripper=p.loadSDF('gripper/wsg50_one_motor_gripper.sdf',globalScaling=1)
	gripper=gripper[0]
	# print(p.getLinkStates(gripper,[0,1,2,3,4,5,6,7,8,9]))
	p.changeDynamics(gripper,-1,mass=0.0001)
	for i in range(9):
		p.changeDynamics(gripper,i,mass=0.0001)
	p.resetBasePositionAndOrientation(gripper,[x,y,z-link1_h-link2_h-head_z-2.5*servo1_z-2.5*servo2_z], p.getQuaternionFromEuler([0,3.14,0]))
	p.createConstraint(multi_head, -1,h_pole1, -1, p.JOINT_FIXED, [0,0,0], [0,0, 0], [0,0,-0.1])
	p.createConstraint(multi_link1, -1,multi_head, -1, p.JOINT_FIXED, [0,0,0], [0,0, l1/2], [0,0,-head_z])
	p.createConstraint(multi_link2, -1,multi_link1, -1, p.JOINT_FIXED, [0,0,0], [0,0, l2/2], [0,0,-l1/2])
	p.createConstraint(multi_servo1, -1,multi_link2, -1, p.JOINT_FIXED, [0,0,0], [0,0, servo1_z], [0,0,-l2/2])
	# p.createConstraint(multi_d_bracket1, -1,multi_servo1, -1, p.JOINT_FIXED, [0,0,0], [0,0, 0], [0,-0.05,-servo1_z])
	p.createConstraint(multi_servo1, -1,multi_servo2, -1, p.JOINT_FIXED, [0,0,0], [0,0, -servo1_z], [0,0,servo2_z])
	p.createConstraint(multi_servo2, -1,gripper, -1, p.JOINT_FIXED, [0,0,0], [0,0, -servo2_z], [0,0,0])

	# p.createConstraint(multi_head, -1, multi_link1, -1, p.JOINT_FIXED, [0,0,0], [0,0, -head_z], [0,0,l1/2])

	return base1,base2