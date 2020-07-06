import pybullet as p
import time
import math
import numpy as np
from datetime import datetime
import pybullet_data

# p.connect(p.GUI)
def import_drop_area(x,y,z=0.5,scale_x=1,scale_y=1,scale_z=0.5):
	p.setAdditionalSearchPath(pybullet_data.getDataPath())
	pick_c=p.createCollisionShape(p.GEOM_BOX,halfExtents=[scale_x,scale_y,scale_z])
	pick_v=p.createVisualShape(p.GEOM_BOX,halfExtents=[scale_x,scale_y,scale_z])
	# p.loadURDF("sphere_small.urdf", [0, 0, 3])
	border1_c=p.createCollisionShape(p.GEOM_BOX,halfExtents=[0.01,scale_y,0.05])
	border1_v=p.createVisualShape(p.GEOM_BOX,halfExtents=[0.01,scale_y,0.05])
	gap=2*scale_y/5.0
	multi_border=p.createMultiBody(baseCollisionShapeIndex=border1_c,baseVisualShapeIndex=border1_v,basePosition=[x+gap/2,y,z+scale_z])
	multi_border=p.createMultiBody(baseCollisionShapeIndex=border1_c,baseVisualShapeIndex=border1_v,basePosition=[x+gap+gap/2,y,z+scale_z])
	multi_border=p.createMultiBody(baseCollisionShapeIndex=border1_c,baseVisualShapeIndex=border1_v,basePosition=[x-gap-gap/2.0,y,z+scale_z])
	multi_border=p.createMultiBody(baseCollisionShapeIndex=border1_c,baseVisualShapeIndex=border1_v,basePosition=[x-gap/2.0,y,z+scale_z])
	multi_border=p.createMultiBody(baseCollisionShapeIndex=border1_c,baseVisualShapeIndex=border1_v,basePosition=[x-scale_x,y,z+scale_z])
	multi_border=p.createMultiBody(baseCollisionShapeIndex=border1_c,baseVisualShapeIndex=border1_v,basePosition=[x+scale_x,y,z+scale_z])

	border1_c=p.createCollisionShape(p.GEOM_BOX,halfExtents=[scale_x,0.01,0.05])
	border1_v=p.createVisualShape(p.GEOM_BOX,halfExtents=[scale_x,0.01,0.05])
	gap=2*scale_x/5.0
	multi_border=p.createMultiBody(baseCollisionShapeIndex=border1_c,baseVisualShapeIndex=border1_v,basePosition=[x,y+gap/2.0,z+scale_z])
	multi_border=p.createMultiBody(baseCollisionShapeIndex=border1_c,baseVisualShapeIndex=border1_v,basePosition=[x,y+gap+gap/2.0,z+scale_z])
	multi_border=p.createMultiBody(baseCollisionShapeIndex=border1_c,baseVisualShapeIndex=border1_v,basePosition=[x,y-gap-gap/2.0,z+scale_z])
	multi_border=p.createMultiBody(baseCollisionShapeIndex=border1_c,baseVisualShapeIndex=border1_v,basePosition=[x,y-gap/2.0,z+scale_z])
	multi_border=p.createMultiBody(baseCollisionShapeIndex=border1_c,baseVisualShapeIndex=border1_v,basePosition=[x,y-scale_y,z+scale_z])
	multi_border=p.createMultiBody(baseCollisionShapeIndex=border1_c,baseVisualShapeIndex=border1_v,basePosition=[x,y+scale_y,z+scale_z])

	multi=p.createMultiBody(baseCollisionShapeIndex=pick_c,baseVisualShapeIndex=pick_v,basePosition=[x,y,z])
