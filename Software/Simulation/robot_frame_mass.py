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
	h_pole_c = p.createCollisionShape(p.GEOM_CYLINDER,height = 2*(scale_x + 0.1)-2*(scale_x/40), radius = 0.015)
	h_pole_v = p.createVisualShape(p.GEOM_CYLINDER,length = 2*(scale_x + 0.1)-2*(scale_x/40), radius = 0.015, rgbaColor=[0,0.39,0,1])
	h_pole1 = p.createMultiBody(baseMass = 0.0001,baseCollisionShapeIndex=h_pole_c,baseVisualShapeIndex=h_pole_v,basePosition=[x,y + 0.03 ,z+scale_z+0.15+0.6+pickAreaHeight+0.02], baseOrientation = p.getQuaternionFromEuler([0, 1.570796,0]))
	h_pole2 = p.createMultiBody(baseMass = 0.0001,baseCollisionShapeIndex=h_pole_c,baseVisualShapeIndex=h_pole_v,basePosition=[x,y - 0.03 ,z+scale_z+0.15+0.6+pickAreaHeight+0.02], baseOrientation = p.getQuaternionFromEuler([0, 1.570796,0]))
	p.createConstraint(base1, -1, pole1, -1, p.JOINT_FIXED, [0,0,1], [0, 0, 0.07], [0,0,-(0.6+pickAreaHeight)/2])
	p.createConstraint(base2, -1, pole2, -1, p.JOINT_FIXED, [0,0,1], [0, 0, 0.07], [0,0,-(0.6+pickAreaHeight)/2])
	p.createConstraint(pole1, -1, upper_base1, -1, p.JOINT_FIXED, [0,0,1], [0, 0, (0.6+pickAreaHeight)/2], [0,0,-0.02])
	p.createConstraint(pole2, -1, upper_base2, -1, p.JOINT_FIXED, [0,0,1], [0, 0, (0.6+pickAreaHeight)/2], [0,0,-0.02])
	p.createConstraint(upper_base1, -1, h_pole1, -1, p.JOINT_FIXED, [0,1,0], [-scale_x/40, 0.03, 0], [0,0,(scale_x + 0.1)-(scale_x/40)])
	p.createConstraint(upper_base1, -1, h_pole2, -1, p.JOINT_FIXED, [0,1,0], [-scale_x/40, -0.03, 0], [0,0,(scale_x + 0.1)-(scale_x/40)])
	p.createConstraint(upper_base2, -1, h_pole1, -1, p.JOINT_FIXED, [0,1,0], [scale_x/40, 0.03, 0], [0,0,-(scale_x + 0.1)+(scale_x/40)])
	p.createConstraint(upper_base2, -1, h_pole2, -1, p.JOINT_FIXED, [0,1,0], [scale_x/40, -0.03, 0], [0,0,-(scale_x + 0.1)+(scale_x/40)])
	return base1,base2
	





