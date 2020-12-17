import pybullet as p
import time
import math
import numpy as np
from datetime import datetime
import pybullet_data

#p.connect(p.GUI)
def MakeArena(x,y,z=0.5,scale_x=0.5,scale_y=1,scale_z=0.5,Inter_area_dist=1,pickAreaHeight=0.9):
	p.setAdditionalSearchPath(pybullet_data.getDataPath())

	pick_c=p.createCollisionShape(p.GEOM_BOX,halfExtents=[scale_x,scale_y,pickAreaHeight/2.0])
	pick_v=p.createVisualShape(p.GEOM_BOX,halfExtents=[scale_x,scale_y,pickAreaHeight/2.0],rgbaColor=[1,0,0,1])
	y_pick = y - scale_y - Inter_area_dist/2.0
	
	drop_c=p.createCollisionShape(p.GEOM_BOX,halfExtents=[scale_x,scale_y,scale_z])
	drop_v=p.createVisualShape(p.GEOM_BOX,halfExtents=[scale_x,scale_y,scale_z],rgbaColor=[1,0,0,1])
	y_drop = y + scale_y + Inter_area_dist/2.0


	border1_c=p.createCollisionShape(p.GEOM_BOX,halfExtents=[0.01,scale_y,0.005])
	border1_v=p.createVisualShape(p.GEOM_BOX,halfExtents=[0.01,scale_y,0.005],rgbaColor=[0,0,1,1])
	gap=2*scale_y/5.0
	multi_border=p.createMultiBody(baseCollisionShapeIndex=border1_c,baseVisualShapeIndex=border1_v,basePosition=[x+gap/2,y_drop,z+scale_z])
	multi_border=p.createMultiBody(baseCollisionShapeIndex=border1_c,baseVisualShapeIndex=border1_v,basePosition=[x+gap+gap/2,y_drop,z+scale_z])
	multi_border=p.createMultiBody(baseCollisionShapeIndex=border1_c,baseVisualShapeIndex=border1_v,basePosition=[x-gap-gap/2.0,y_drop,z+scale_z])
	multi_border=p.createMultiBody(baseCollisionShapeIndex=border1_c,baseVisualShapeIndex=border1_v,basePosition=[x-gap/2.0,y_drop,z+scale_z])
	multi_border=p.createMultiBody(baseCollisionShapeIndex=border1_c,baseVisualShapeIndex=border1_v,basePosition=[x-scale_x,y_drop,z+scale_z])
	multi_border=p.createMultiBody(baseCollisionShapeIndex=border1_c,baseVisualShapeIndex=border1_v,basePosition=[x+scale_x,y_drop,z+scale_z])

	border1_c=p.createCollisionShape(p.GEOM_BOX,halfExtents=[scale_x,0.01,0.005])
	border1_v=p.createVisualShape(p.GEOM_BOX,halfExtents=[scale_x,0.01,0.005],rgbaColor=[0,0,1,1])
	gap=2*scale_x/5.0
	multi_border=p.createMultiBody(baseCollisionShapeIndex=border1_c,baseVisualShapeIndex=border1_v,basePosition=[x,y_drop+gap/2.0,z+scale_z])
	multi_border=p.createMultiBody(baseCollisionShapeIndex=border1_c,baseVisualShapeIndex=border1_v,basePosition=[x,y_drop+gap+gap/2.0,z+scale_z])
	multi_border=p.createMultiBody(baseCollisionShapeIndex=border1_c,baseVisualShapeIndex=border1_v,basePosition=[x,y_drop-gap-gap/2.0,z+scale_z])
	multi_border=p.createMultiBody(baseCollisionShapeIndex=border1_c,baseVisualShapeIndex=border1_v,basePosition=[x,y_drop-gap/2.0,z+scale_z])
	multi_border=p.createMultiBody(baseCollisionShapeIndex=border1_c,baseVisualShapeIndex=border1_v,basePosition=[x,y_drop-scale_y,z+scale_z])
	multi_border=p.createMultiBody(baseCollisionShapeIndex=border1_c,baseVisualShapeIndex=border1_v,basePosition=[x,y_drop+scale_y,z+scale_z])

	multi=p.createMultiBody(baseCollisionShapeIndex=pick_c,baseVisualShapeIndex=pick_v,basePosition=[x,y_pick,pickAreaHeight/2])

	multi=p.createMultiBody(baseCollisionShapeIndex=drop_c,baseVisualShapeIndex=drop_v,basePosition=[x,y_drop,z])

if __name__=='__main__':
	p.connect(p.GUI)
	MakeArena(x=0,y=0,z=-0.15,scale_x=0.3,scale_y=0.3,scale_z=0,Inter_area_dist=0.5,pickAreaHeight=0.5)
	p.resetDebugVisualizerCamera(1.3 , 0, -41, [0, 0, 0.5])
	time.sleep(10)