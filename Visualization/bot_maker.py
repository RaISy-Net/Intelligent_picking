import pybullet as p
import time
import math
import numpy as np
from datetime import datetime
import pybullet_data

# p.connect(p.GUI)
def MakeBot(x,y,z,l1=0.5,l2=0.5):
	p.setAdditionalSearchPath(pybullet_data.getDataPath())
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
	servo1_v=p.createVisualShape(p.GEOM_BOX,halfExtents=[servo1_x,servo1_y,servo1_z],rgbaColor=[1,0,0,1])

	d_bracket1_x=0.002
	d_bracket1_y=0.002
	d_bracket1_z=0.002

	d_bracket1_c=p.createCollisionShape(p.GEOM_MESH,fileName='d_bracket.stl',meshScale=[d_bracket1_x,d_bracket1_y,d_bracket1_z])
	d_bracket1_v=p.createVisualShape(p.GEOM_MESH,fileName='d_bracket.stl',meshScale=[d_bracket1_x,d_bracket1_y,d_bracket1_z],rgbaColor=[0,0,0,1])

	servo2_x=0.03
	servo2_y=0.05
	servo2_z=0.02
	servo2_c=p.createCollisionShape(p.GEOM_BOX,halfExtents=[servo2_x,servo2_y,servo2_z])
	servo2_v=p.createVisualShape(p.GEOM_BOX,halfExtents=[servo2_x,servo2_y,servo2_z],rgbaColor=[1,0,0,1])

	d_bracket2_x=0.002
	d_bracket2_y=0.002
	d_bracket2_z=0.002

	d_bracket2_c=p.createCollisionShape(p.GEOM_MESH,fileName='d_bracket2.stl',meshScale=[d_bracket2_x,d_bracket2_y,d_bracket2_z])
	d_bracket2_v=p.createVisualShape(p.GEOM_MESH,fileName='d_bracket2.stl',meshScale=[d_bracket2_x,d_bracket2_y,d_bracket2_z],rgbaColor=[0,0,0,1])
	


	multi_head=p.createMultiBody(baseCollisionShapeIndex=head_c,baseVisualShapeIndex=head_v,basePosition=[x,y,z])
	multi_link1=p.createMultiBody(baseCollisionShapeIndex=link1_c,baseVisualShapeIndex=link1_v,basePosition=[x,y,z-link1_h/2.0-head_z])
	multi_link2=p.createMultiBody(baseCollisionShapeIndex=link2_c,baseVisualShapeIndex=link2_v,basePosition=[x,y,z-link2_h/2.0-head_z-link1_h])
	multi_servo1=p.createMultiBody(baseCollisionShapeIndex=servo1_c,baseVisualShapeIndex=servo1_v,basePosition=[x,y,z-link1_h-link2_h-head_z-servo1_z])
	multi_d_bracket1=p.createMultiBody(baseCollisionShapeIndex=d_bracket1_c,baseVisualShapeIndex=d_bracket1_v,basePosition=[x-0.01,y,z-link1_h-link2_h-head_z-2.5*servo1_z])
	multi_servo2=p.createMultiBody(baseCollisionShapeIndex=servo2_c,baseVisualShapeIndex=servo2_v,basePosition=[x,y,z-link1_h-link2_h-head_z-2.5*servo1_z-servo2_z])
	multi_d_bracket2=p.createMultiBody(baseCollisionShapeIndex=d_bracket2_c,baseVisualShapeIndex=d_bracket2_v,basePosition=[x-0.01,y,z-link1_h-link2_h-head_z-2.5*servo1_z-2.5*servo2_z])

	gripper=p.loadSDF('gripper/wsg50_one_motor_gripper.sdf',globalScaling=1)
	gripper=gripper[0]
	p.resetBasePositionAndOrientation(gripper,[x,y,z-link1_h-link2_h-head_z-2.5*servo1_z-2.5*servo2_z], p.getQuaternionFromEuler([0,3.14,0]))
	# multi=p.createMultiBody(baseCollisionShapeIndex=drop_c,baseVisualShapeIndex=drop_v,basePosition=[x,y_drop,z])

