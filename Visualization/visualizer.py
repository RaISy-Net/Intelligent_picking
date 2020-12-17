'''
rough script to visualize

'''




import pybullet as p
import time
import math
import numpy as np
from datetime import datetime
import pybullet_data

p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf", [0, 0, 0])
p.setGravity(0, 0, -10)



#trailDuration is duration (in seconds) after debug lines will be removed automatically
#use 0 for no-removal
#trailDuration = 15
def draw_frame(point,draw_target=False,frame_length=0.3):
  if draw_target:
    p.addUserDebugLine(point,[point[0]+frame_length,point[1],point[2]],[0,1,0],5)
    p.addUserDebugLine(point,[point[0],point[1]+frame_length,point[2]],[0,1,0],5)
    p.addUserDebugLine(point,[point[0],point[1],point[2]+frame_length],[0,1,0],5)
  else:
    p.addUserDebugLine(point,[point[0]+frame_length,point[1],point[2]],[1,0,0],5)
    p.addUserDebugLine(point,[point[0],point[1]+frame_length,point[2]],[0,1,0],5)
    p.addUserDebugLine(point,[point[0],point[1],point[2]+frame_length],[0,0,1],5)



grip_list= p.loadSDF("./kukka_wsg50/kuka_with_wsg50.sdf")
gripper1 = grip_list[0]
p.resetBasePositionAndOrientation(gripper1,[0,0,0],p.getQuaternionFromEuler([0,0,1.57]))



radius = 0.6
reverse = False
gripper_centre_index = 8
t = 0.
#for i in range(p.getNumJoints(gripper1)):
#  print('\nJoint_',i,"info:",p.getJointInfo(gripper1,i))

motor_fing = np.pi/4
hinge_fing = -np.pi/4
#grab  config
grab = np.array([motor_fing,hinge_fing,-motor_fing,hinge_fing])

#params for to and fro grabbing
grab_sim_steps = 20
grab_r = False
grab_counter = 0
hasprev = 0

#simulation
while 1:
  
  pos_1 = [-radius*math.cos(t), radius*math.sin(t) ,0.5 ]
  #draw_frame(p.getLinkState(gripper1,gripper_centre_index)[0],frame_length=0.1)
  #draw_frame(pos_1,frame_length=0.5)

  angles_1 = p.calculateInverseKinematics(gripper1,gripper_centre_index,pos_1)


  p.setJointMotorControlArray(gripper1,[0,1,2,3,4,5,6,10],
                              controlMode=p.POSITION_CONTROL,
                              targetPositions=angles_1[0:8])
  
  if(grab_counter<grab_sim_steps and grab_r==False):
    p.setJointMotorControlArray(gripper1,[11,12,14,15],controlMode=p.POSITION_CONTROL,
                                                     targetPositions=np.zeros(len(grab)))
    grab_counter+=1

  elif(grab_counter>0):
    p.setJointMotorControlArray(gripper1,[11,12,14,15],controlMode=p.POSITION_CONTROL,
                                                     targetPositions=grab)
    grab_r=True
    grab_counter-=1
  else:
    grab_r = False


  if(hasprev == 1):
    p.addUserDebugLine(prevPose, pos_1, [0, 0, 0.3], 1)
  prevPose = pos_1

  if(t<=np.pi and reverse!=True):
    t+=0.01
  elif(t>0):
    reverse = True
    t-=0.01
  else:
    reverse = False

  p.stepSimulation()
  hasprev = 1
p.disconnect()
