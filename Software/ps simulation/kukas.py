import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

import pybullet as p
import numpy as np
import copy
import math
import pybullet_data
from drop_area import *


class Kuka:

  def __init__(self, urdfRootPath=pybullet_data.getDataPath(), timeStep=0.01):
    self.urdfRootPath = urdfRootPath
    self.timeStep = timeStep
    self.maxVelocity = .35
    self.maxForce = 200.
    self.fingerAForce = 2
    self.fingerBForce = 2.5
    self.fingerTipForce = 2
    self.useInverseKinematics = 1
    self.useSimulation = 1
    self.useNullSpace = 1
    self.useOrientation = 1
    self.kukaEndEffectorIndex = 8
    self.Area_Halfdim = 0.25
    #lower limits for null space
    self.ll = [-.967, -2, -2.96, 0.19, -2.96, -2.09, -3.05]
    #upper limits for null space
    self.ul = [.967, 2, 2.96, 2.29, 2.96, 2.09, 3.05]
    #joint ranges for null space
    self.jr = [5.8, 4, 5.8, 4, 5.8, 4, 6]
    #restposes for null space
    self.rp = [0, 0, 0, 0.5 * math.pi, 0, -math.pi * 0.5 * 0.66, 0]
    #joint damping coefficents
    self.jd = [
        0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001,
        0.00001, 0.00001, 0.00001, 0.00001
    ]
    self.reset()

  def reset(self):
    self.kukaUid = p.loadSDF("./kukka_wsg50/kuka_with_wsg50.sdf")[0]
    for i in range (p.getNumJoints(self.kukaUid)):
      print(p.getJointInfo(self.kukaUid,i))
    p.resetBasePositionAndOrientation(self.kukaUid, [-0.100000, 0.000000, 0.070000],
                                      [0.000000, 0.000000, 0.000000, 1.000000])
    self.jointPositions = [
        -1.5700000, 0.07240780636452046, -0.0010198792237838638, 0.008887538067466616, 0.000978788934680996, 2.0943999489061773, -0.4182119641789541]
    
    self.numJoints = p.getNumJoints(self.kukaUid)
    self.endEffectorPos = [0, -0.5, 1]
    self.endEffectorAngle = 0
    for jointIndex in range(7):
      p.resetJointState(self.kukaUid, jointIndex, self.jointPositions[jointIndex])
      p.setJointMotorControl2(self.kukaUid,
                              jointIndex,
                              p.POSITION_CONTROL,
                              targetPosition=self.jointPositions[jointIndex],
                                force=self.maxForce)
    #for _ in range(10000):
        #self.applyAction([0,0,0.001,0,[0,0,0,0]])
        #p.stepSimulation()
        #if p.getLinkState(self.kukaUid,self.kukaEndEffectorIndex)[0][2]>1.1:
            #for i in range (7):
             #   print(p.getJointState(self.kukaUid,i)[0])
              #  self.jointPositions[i]=p.getJointState(self.kukaUid,i)[0]
            #print("doneeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee")
            #break
    #print(self.jointPositions)
    #self.jointPositions[0]=-1.780000
    #for jointIndex in range(7):
      #p.resetJointState(self.kukaUid, jointIndex, self.jointPositions[jointIndex])
      #p.setJointMotorControl2(self.kukaUid,
         #                     jointIndex,
          #                    p.POSITION_CONTROL,
           #                   targetPosition=self.jointPositions[jointIndex],
            #                    force=self.maxForce)

    
    motor_fing = np.pi/4
    hinge_fing = -np.pi/4
    #grab  config
    grab = np.array([motor_fing,hinge_fing,-motor_fing,hinge_fing])

    MakeArena(x=0,y=0,z=-0.15,
	      scale_x=self.Area_Halfdim,scale_y=self.Area_Halfdim,scale_z=0,
	      Inter_area_dist=0.5,pickAreaHeight=0.5)

    
    

    self.motorNames = []
    self.motorIndices = []

    for i in range(self.numJoints):
      jointInfo = p.getJointInfo(self.kukaUid, i)
      qIndex = jointInfo[3]
      if qIndex > -1:
        #print("motorname")
        #print(jointInfo[1])
        self.motorNames.append(str(jointInfo[1]))
        self.motorIndices.append(i)

  def getActionDimension(self):
    if (self.useInverseKinematics):
      return len(self.motorIndices)
    return 6  #position x,y,z and roll/pitch/yaw euler angles of end effector

  def getObservationDimension(self):
    return len(self.getObservation())

  def getObservation(self):
    observation = []
    state = p.getLinkState(self.kukaUid, self.kukaGripperIndex)
    pos = state[0]
    orn = state[1]
    euler = p.getEulerFromQuaternion(orn)

    observation.extend(list(pos))
    observation.extend(list(euler))

    return observation

  def applyAction(self, motorCommands):

    #print ("self.numJoints")
    #print (self.numJoints)
    if (self.useInverseKinematics):

      dx = motorCommands[0]
      dy = motorCommands[1]
      dz = motorCommands[2]
      da = motorCommands[3]
      grab = motorCommands[4]

      state = p.getLinkState(self.kukaUid, self.kukaEndEffectorIndex)
      actualEndEffectorPos = state[0]
      #print("pos[2] (getLinkState(kukaEndEffectorIndex)")
      #print(actualEndEffectorPos[2])

      self.endEffectorPos[0] = self.endEffectorPos[0] + dx
      #if (self.endEffectorPos[0] > 0.70):
        #self.endEffectorPos[0] = 0.70
      #if (self.endEffectorPos[0] < 0.45):
        #self.endEffectorPos[0] = 0.45
      self.endEffectorPos[1] = self.endEffectorPos[1] + dy
      #if (self.endEffectorPos[1] < -0.20):
        #self.endEffectorPos[1] = -0.20
      #if (self.endEffectorPos[1] > 0.24):
        #self.endEffectorPos[1] = 0.24

      #print ("self.endEffectorPos[2]")
      #print (self.endEffectorPos[2])
      #print("actualEndEffectorPos[2]")
      #print(actualEndEffectorPos[2])
      #if (dz<0 or actualEndEffectorPos[2]<0.5):
      self.endEffectorPos[2] = self.endEffectorPos[2] + dz

      self.endEffectorAngle = self.endEffectorAngle + da
      pos = self.endEffectorPos
      orn = p.getQuaternionFromEuler([0, -math.pi, 0])  # -math.pi,yaw])
      if (self.useNullSpace == 1):
        if (self.useOrientation == 1):
          jointPoses = p.calculateInverseKinematics(self.kukaUid, self.kukaEndEffectorIndex, pos,
                                                    orn)
        else:
          jointPoses = p.calculateInverseKinematics(self.kukaUid,
                                                    self.kukaEndEffectorIndex,
                                                    pos)
      else:
        if (self.useOrientation == 1):
          jointPoses = p.calculateInverseKinematics(self.kukaUid,
                                                    self.kukaEndEffectorIndex,
                                                    pos,
                                                    orn)
        else:
          jointPoses = p.calculateInverseKinematics(self.kukaUid, self.kukaEndEffectorIndex, pos)

      #print("jointPoses")
      #print(jointPoses)
      #print("self.kukaEndEffectorIndex")
      #print(self.kukaEndEffectorIndex)
      if (self.useSimulation):
        #for i in range(self.kukaEndEffectorIndex + 1):
          #print(i)
        p.setJointMotorControlArray(self.kukaUid,
                                  [0,1,2,3,4,5,6,10],
                                  controlMode=p.POSITION_CONTROL,
                                  targetPositions=jointPoses[0:8],
                                  targetVelocities=np.zeros((8)))
                                  #force=self.maxForce,
                                  #maxVelocity=self.maxVelocity,
                                  #positionGain=0.3,
                                  #velocityGain=1)
      else:
        #reset the joint state (ignoring all dynamics, not recommended to use during simulation)
        for i in range(self.numJoints):
          p.resetJointState(self.kukaUid, i, jointPoses[i])
      #fingers
      #p.setJointMotorControl2(self.kuka_gripper,
       #                       0,
        #                      p.POSITION_CONTROL,
         #                     targetPosition=self.endEffectorAngle,
          #                    force=self.maxForce)
      p.setJointMotorControlArray(self.kukaUid,[11,12,14,15],controlMode=p.POSITION_CONTROL,
                                                     targetPositions=grab)
      p.setJointMotorControlArray(self.kukaUid,[13,16],controlMode=p.POSITION_CONTROL,targetPositions=[0,0])
          
    else:
      for action in range(len(motorCommands)):
        motor = self.motorIndices[action]
        p.setJointMotorControl2(self.kukaUid,
                                motor,
                                p.POSITION_CONTROL,
                                targetPosition=motorCommands[action],
                                force=self.maxForce)
