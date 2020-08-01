import pybullet as p
import time
import math
from datetime import datetime

#clid = p.connect(p.SHARED_MEMORY)
import pybullet_data

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")



c1=p.loadURDF("cube_small.urdf", [0,0,1])
c2=p.loadURDF("cube_small.urdf", [0, 0, 1.5])
p.createConstraint(c2,-1,-1,-1,p.JOINT_PRISMATIC,[0,0,1],[0,0,0],[0,0,0])
p.createConstraint(c1,-1,-1,-1,p.JOINT_PRISMATIC,[0,0,1],[0,0,0],[0,0,0])
p.changeDynamics(c2,-1,mass=10)
# p.setGravity(0,0,-10)
# p.loadURDF("cube_small.urdf", [2, -2, 5])
print(p.getBasePositionAndOrientation(c2))

while (1):
	pos,orn=p.getBasePositionAndOrientation(c2)
	euler_orn=p.getEulerFromQuaternion(orn)
	print(euler_orn)
	p.applyExternalForce(c1,-1,[euler_orn[0],euler_orn[1],1],[pos[0],pos[1],pos[2]],p.WORLD_FRAME)
	p.stepSimulation()
	time.sleep(0.01)
	