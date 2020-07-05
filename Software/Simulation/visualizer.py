import pybullet as p
import time
import math
import numpy as np
from datetime import datetime
import pybullet_data

clid = p.connect(p.SHARED_MEMORY)
if (clid < 0):
  p.connect(p.GUI)
  #p.connect(p.SHARED_MEMORY_GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())




p.loadURDF("plane.urdf", [0, 0, 0])

ll = [-.967, -2, -2.96, 0.19, -2.96, -2.09, -3.05]
    #upper limits for null space
ul = [.967, 2, 2.96, 2.29, 2.96, 2.09, 3.05]
    #joint ranges for null space
jr = [5.8, 4, 5.8, 4, 5.8, 4, 6]
    #restposes for null space
rp = [0, 0, 0, 0.5 * math.pi, 0, -math.pi * 0.5 * 0.66, 0]
    #joint damping coefficents
jd = [
        0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001,
        0.00001, 0.00001, 0.00001, 0.00001
    ]


kukaEndEffectorIndex = 14

p.setGravity(0, 0, -10)
t = 0.
prevPose = [0, 0, 0]
prevPose1 = [0, 0, 0]
hasPrevPose = 0
useNullSpace = 1

useOrientation = 1
#If we set useSimulation=0, it sets the arm pose to be the IK result directly without using dynamic control.
#This can be used to test the IK result accuracy.
useSimulation = 1
useRealTimeSimulation = 0
ikSolver = 0
#trailDuration is duration (in seconds) after debug lines will be removed automatically
#use 0 for no-removal
#trailDuration = 15
def draw_frame(point,draw_target=False):
  if draw_target:
    p.addUserDebugLine(point,[point[0]+0.5,point[1],point[2]],[0,1,0],5)
    p.addUserDebugLine(point,[point[0],point[1]+0.5,point[2]],[0,1,0],5)
    p.addUserDebugLine(point,[point[0],point[1],point[2]+0.5],[0,1,0],5)
  else:
    p.addUserDebugLine(point,[point[0]+0.3,point[1],point[2]],[1,0,0],5)
    p.addUserDebugLine(point,[point[0],point[1]+0.3,point[2]],[0,1,0],5)
    p.addUserDebugLine(point,[point[0],point[1],point[2]+0.3],[0,0,1],5)

i=0
radius = 0.6
reverse = False
grip_list= p.loadSDF("./kukka_wsg50/kuka_with_wsg50.sdf")
gripper1 = grip_list[0]
p.resetBasePositionAndOrientation(gripper1,[0,0,0],p.getQuaternionFromEuler([0,0,1.57]))

grip_list2= p.loadSDF("./kukka_wsg50/kuka_with_wsg50.sdf")
gripper2 = grip_list2[0]
p.resetBasePositionAndOrientation(gripper2,[1,0,0],p.getQuaternionFromEuler([0,0,1.57]))
while 1:
  
  #pos = [-radius*math.cos(t), radius*math.sin(t) ,0.5 ]
  pos_1 = [0,0.7,0.5]
  pos_2 = [1.5,0,0.5]
  draw_frame(pos_1)
  angles_1 = p.calculateInverseKinematics(gripper1,11,pos_1)
  angles_2 = p.calculateInverseKinematics(gripper2,12,pos_2)

  p.setJointMotorControlArray(gripper1,np.arange(13),controlMode=p.POSITION_CONTROL,
                                                     targetPositions=angles_1)
  p.setJointMotorControlArray(gripper2,np.arange(13),controlMode=p.POSITION_CONTROL,targetPositions=angles_2)

  if (t!=0.0):
    p.addUserDebugLine(prevPose, pos_1, [0, 0, 0.3], 1)
  prevPose = pos_1

  if(t<=0.5 and reverse!=True):
    t+=0.01
  elif(t>0):
    reverse = True
    t-=0.01
  else:
    reverse = False

  p.stepSimulation()
p.disconnect()
