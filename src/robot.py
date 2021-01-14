import os
import pdb
import cv2
import time
import glob
import random
import numpy as np
import pybullet as p
import pybullet_data
import distutils.dir_util
from pkg_resources import parse_version
from src.drop_area import *
import matplotlib.pyplot as plt


class robot:
	def __init__(self, urdfRoot = pybullet_data.getDataPath(), num_Objects = 25, blockRandom = 0.3):		
		#connecting to the simulation
		p.connect(p.GUI)
		p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"), 0, 0, 0)
		
		#loading the robot parts
		self.bot = p.loadURDF('./src/rsc/bot.urdf',basePosition = [0,0,1.7])
		self.rail1 = p.loadURDF('./src/rsc/rail1.urdf',basePosition = [-1.25,0,0.01],baseOrientation = p.getQuaternionFromEuler([0,0,np.pi/2]), useFixedBase = True)
		self.rail2 = p.loadURDF('./src/rsc/rail1.urdf',basePosition = [1.25,0,0.01],baseOrientation = p.getQuaternionFromEuler([0,0,np.pi/2]), useFixedBase = True)
		self.cam1 = p.loadURDF('./src/rsc/cam1.urdf',basePosition = [1.5,-1,2],baseOrientation = p.getQuaternionFromEuler([0,0,np.pi/2]),useFixedBase = True)
		self.cam2 = p.loadURDF('./src/rsc/cam1.urdf',basePosition = [1.5,1,2],baseOrientation = p.getQuaternionFromEuler([0,0,np.pi/2]),useFixedBase = True)
		
		p.setGravity(0,0,-10)
		#defining the link numbers
		self.n = p.getNumJoints(self.bot)
		self.wrist = 11
		self.mid_arm = 8
		self.upper_arm = 5
		self.head = 0
		self.plate_left = 16
		self.plate_right = 14
		self.servo = 13
		self.camera = 20
		self.end_effect = 13
		self.suction = 25
		self.suction_cup = 25
		self.cart2_link = 44
		self.cart1_link = 70
		i = 0
		
		#constraining the rails to increase the stability of the bot
		p.createConstraint(self.rail1,-1,-1,-1,p.JOINT_FIXED,[1,0,0],[0,0,0],[-1.25,0,0.004989748675026239],childFrameOrientation=p.getQuaternionFromEuler([0,0,np.pi/2]))
		p.createConstraint(self.rail2,-1,-1,-1,p.JOINT_FIXED,[1,0,0],[0,0,0],[ 1.25,0,0.004989748675026239],childFrameOrientation=p.getQuaternionFromEuler([0,0,np.pi/2]))
		p.changeDynamics(bodyUniqueId=self.rail1,
				             linkIndex=-1,
				             lateralFriction=0.6)
		p.changeDynamics(bodyUniqueId=self.rail2,
				             linkIndex=-1,
				             lateralFriction=0.6)
		self.n = p.getNumJoints(self.bot)

		#changing the friction values of the wheels of the carts
		wheels = [46,49,52,55,81,84,87,90]
		for i in wheels:
			p.changeDynamics(bodyUniqueId=self.bot,
				             linkIndex=i,
				             lateralFriction=0.7,
				             restitution=0.5)

		#changing the friction and restitution values of the fingers
		fingers = [14,15]
		for i in fingers:
			p.changeDynamics(bodyUniqueId=self.bot,
				             linkIndex=i,
				             lateralFriction=2,
				             restitution=0.5)
		
		for _ in range(500):
			p.stepSimulation()
		p.setPhysicsEngineParameter(numSolverIterations=150)

		#setting arena and overhead camera parameters
		self.Area_Halfdim = 1
		self._blockRandom = blockRandom
		self._urdfRoot = urdfRoot
		self._width = 1024
		self._height = 1024

		#loading the arena
		MakeArena(x=0,y=0,z=0.05,
	      scale_x=self.Area_Halfdim,scale_y=self.Area_Halfdim,scale_z=0,
	      Inter_area_dist=0.5,pickAreaHeight=0.90)
		
		self.overhead_camera(1)

		#loading various objects
		self._numObjects = num_Objects
		urdfList = self.get_objects()
		self._objectUids = self._place_objects(urdfList)

		for _ in range(500):
			p.stepSimulation()
		
		#taking overhead camera image
		img = self.overhead_camera(0)
		img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
		cv2.imwrite("./src/objdet_images/image.jpeg", img)
		p.resetDebugVisualizerCamera(4, 0, -40, [0,0,0])

	#function to reset the environment if required
	def reset(self,a):		
		p.resetSimulation()
		p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"), 0, 0, 0)
		
		#loading the robot parts
		self.bot = p.loadURDF('./src/rsc/bot.urdf',basePosition = [0,0,1.7])
		self.rail1 = p.loadURDF('./src/rsc/rail1.urdf',basePosition = [-1.25,0,0.01],baseOrientation = p.getQuaternionFromEuler([0,0,np.pi/2]), useFixedBase = True)
		self.rail2 = p.loadURDF('./src/rsc/rail1.urdf',basePosition = [1.25,0,0.01],baseOrientation = p.getQuaternionFromEuler([0,0,np.pi/2]), useFixedBase = True)
		self.cam1 = p.loadURDF('./src/rsc/cam1.urdf',basePosition = [1.5,-1,2],baseOrientation = p.getQuaternionFromEuler([0,0,np.pi/2]),useFixedBase = True)
		self.cam2 = p.loadURDF('./src/rsc/cam1.urdf',basePosition = [1.5,1,2],baseOrientation = p.getQuaternionFromEuler([0,0,np.pi/2]),useFixedBase = True)
		
		p.setGravity(0,0,-10)
		#defining the link numbers
		self.n = p.getNumJoints(self.bot)
		self.wrist = 11
		self.mid_arm = 8
		self.upper_arm = 5
		self.head = 0
		self.plate_left = 16
		self.plate_right = 14
		self.servo = 13
		self.camera = 20
		self.end_effect = 13
		self.suction = 25
		self.suction_cup = 25
		self.cart2_link = 44
		self.cart1_link = 70
		i = 0
		
		#constraining the rails to increase the stability of the bot
		p.createConstraint(self.rail1,-1,-1,-1,p.JOINT_FIXED,[1,0,0],[0,0,0],[-1.25,0,0.004989748675026239],childFrameOrientation=p.getQuaternionFromEuler([0,0,np.pi/2]))
		p.createConstraint(self.rail2,-1,-1,-1,p.JOINT_FIXED,[1,0,0],[0,0,0],[ 1.25,0,0.004989748675026239],childFrameOrientation=p.getQuaternionFromEuler([0,0,np.pi/2]))
		p.changeDynamics(bodyUniqueId=self.rail1,
				             linkIndex=-1,
				             lateralFriction=0.6)
		p.changeDynamics(bodyUniqueId=self.rail2,
				             linkIndex=-1,
				             lateralFriction=0.6)
		self.n = p.getNumJoints(self.bot)

		#changing the friction values of the wheels of the carts
		wheels = [46,49,52,55,81,84,87,90]
		for i in wheels:
			p.changeDynamics(bodyUniqueId=self.bot,
				             linkIndex=i,
				             lateralFriction=0.7,
				             restitution=0.5)

		#changing the friction and restitution values of the fingers
		fingers = [14,15]
		for i in fingers:
			p.changeDynamics(bodyUniqueId=self.bot,
				             linkIndex=i,
				             lateralFriction=2,
				             restitution=0.5)
		
		for _ in range(500):
			p.stepSimulation()
		p.setPhysicsEngineParameter(numSolverIterations=150)

		#setting arena and overhead camera parameters
		self.Area_Halfdim = 1
		self._blockRandom = blockRandom
		self._urdfRoot = urdfRoot
		self._width = 1024
		self._height = 1024

		#loading the arena
		MakeArena(x=0,y=0,z=0.05,
	      scale_x=self.Area_Halfdim,scale_y=self.Area_Halfdim,scale_z=0,
	      Inter_area_dist=0.5,pickAreaHeight=0.90)
		
		self.overhead_camera(1)

		#loading various objects
		self._numObjects = num_Objects
		urdfList = self.get_objects()
		self._objectUids = self._place_objects(urdfList)

		for _ in range(500):
			p.stepSimulation()
		
		#taking overhead camera image
		img = self.overhead_camera(0)
		img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
		cv2.imwrite("./src/objdet_images/image"+str(a)+".jpeg", img)
		p.resetDebugVisualizerCamera(4, 0, -40, [0,0,0])
	
	#function to extend arm fully
	def extend_arm(self, xpos, ypos):
		i = 0
		j = 0
		k = 0
		cam = 1 
		currentPos_init_1 = p.getJointState(self.bot, self.upper_arm)
		currentPos_1 = p.getJointState(self.bot, self.upper_arm)
		currentPos_init_2 = p.getJointState(self.bot, self.mid_arm)
		currentPos_2 = p.getJointState(self.bot, self.mid_arm)
		currentPos_init_3 = p.getJointState(self.bot, self.wrist)
		currentPos_3 = p.getJointState(self.bot, self.wrist)
		while(1):
			p.setJointMotorControl2(self.bot, self.upper_arm,p.POSITION_CONTROL, targetPosition = currentPos_1[0]-(i/100))
			p.setJointMotorControl2(self.bot, self.mid_arm,p.POSITION_CONTROL, targetPosition = currentPos_2[0]-(j/100))
			p.setJointMotorControl2(self.bot, self.wrist,p.POSITION_CONTROL, targetPosition = currentPos_3[0]-(k/100))
			p.stepSimulation()
			time.sleep(1./240.)
			i = i+0.02
			j = j+0.02
			k = k+0.02
			currentPos_1 = p.getJointState(self.bot, self.upper_arm)
			currentPos_2 = p.getJointState(self.bot, self.mid_arm)
			currentPos_3 = p.getJointState(self.bot, self.wrist)
			if cam==1 and currentPos_1[0]<-0.24:
				z = self.end_effector()[0][2]
				p.resetDebugVisualizerCamera(0.4, 180, -20, [xpos, ypos, z-0.05])
				cam = 0
			if currentPos_1[0]<-0.28:
				i=0
				p.setJointMotorControl2(self.bot, self.upper_arm,p.VELOCITY_CONTROL, targetVelocity = 0)
			if currentPos_2[0]<-0.28:
				j=0
				p.setJointMotorControl2(self.bot, self.mid_arm,p.VELOCITY_CONTROL, targetVelocity = 0)
			if currentPos_3[0]<-0.28:
				k=0
				p.setJointMotorControl2(self.bot, self.wrist,p.VELOCITY_CONTROL, targetVelocity = 0)
			if (i==0 and j==0 and k==0):
				break

	#function to contract the arm fully
	def contract_arm(self):
		i = 0
		j = 0
		k = 0
		p.resetDebugVisualizerCamera(2, 180, -41, [0, 1.4, 0.2])
		currentPos_init_1 = p.getJointState(self.bot, self.upper_arm)
		currentPos_1 = p.getJointState(self.bot, self.upper_arm)
		currentPos_init_2 = p.getJointState(self.bot, self.mid_arm)
		currentPos_2 = p.getJointState(self.bot, self.mid_arm)
		currentPos_init_3 = p.getJointState(self.bot, self.wrist)
		currentPos_3 = p.getJointState(self.bot, self.wrist)
		while(1):
			p.setJointMotorControl2(self.bot, self.upper_arm,p.POSITION_CONTROL, targetPosition = currentPos_1[0]+(i/100))
			p.setJointMotorControl2(self.bot, self.mid_arm,p.POSITION_CONTROL, targetPosition = currentPos_2[0]+(j/100))
			p.setJointMotorControl2(self.bot, self.wrist,p.POSITION_CONTROL, targetPosition = currentPos_3[0]+(k/100))
			p.stepSimulation()
			time.sleep(1./240.)
			i = i+0.02
			j = j+0.02
			k = k+0.02
			currentPos_1 = p.getJointState(self.bot, self.upper_arm)
			currentPos_2 = p.getJointState(self.bot, self.mid_arm)
			currentPos_3 = p.getJointState(self.bot, self.wrist)
			if currentPos_1[0]>0:
				i=0
				p.setJointMotorControl2(self.bot, self.upper_arm,p.VELOCITY_CONTROL, targetVelocity = 0)
			if currentPos_2[0]>0:
				j=0
				p.setJointMotorControl2(self.bot, self.mid_arm,p.VELOCITY_CONTROL, targetVelocity = 0)
			if currentPos_3[0]>0:
				k=0
				p.setJointMotorControl2(self.bot, self.wrist,p.VELOCITY_CONTROL, targetVelocity = 0)
			if (i==0 and j==0 and k==0):
				break
	
	#function to extend the wrist only
	def extend_wrist(self, size):
		i = 0
		currentPos_init = p.getJointState(self.bot, self.wrist)
		currentPos = p.getJointState(self.bot, self.wrist)
		while(currentPos[0]>currentPos_init[0]-size):
			currentPos = p.getJointState(self.bot, self.wrist)
			p.setJointMotorControl2(self.bot, self.wrist,p.POSITION_CONTROL, targetPosition = currentPos[0]-(i/100))
			p.stepSimulation()
			time.sleep(1./240.)
			i = i+0.001
		p.setJointMotorControl2(self.bot, self.wrist,p.VELOCITY_CONTROL, targetVelocity = 0)
	
	#function to contract the wrist 
	def contract_wrist(self, size):
		i = 0
		currentPos_init = p.getJointState(self.bot, self.wrist)
		currentPos = p.getJointState(self.bot, self.wrist)
		while(currentPos[0]<currentPos_init[0] + size):
			currentPos = p.getJointState(self.bot, self.wrist)
			p.setJointMotorControl2(self.bot, self.wrist,p.POSITION_CONTROL, targetPosition = currentPos[0]+(i/100))
			p.stepSimulation()
			time.sleep(1./240.)
			i = i+0.001
		p.setJointMotorControl2(self.bot, self.wrist,p.VELOCITY_CONTROL, targetVelocity = 0)

	#function to move the head only -- not used in our sim
	def move_head(self, pos):
		i = 0
		pos = -pos
		currentPos = p.getJointState(self.bot, self.head)
		if(currentPos[0] < pos):
			while(currentPos[0]< pos):
				currentPos = p.getJointState(self.bot, self.head)
				p.setJointMotorControl2(self.bot, self.head,p.POSITION_CONTROL, targetPosition = currentPos[0]+(i/100), velocityGain = 1)
				p.stepSimulation()
				time.sleep(1./240.)
				i = i+0.01
		if(currentPos[0] > pos):
			while(currentPos[0] > pos):
				currentPos = p.getJointState(self.bot, self.head)
				p.setJointMotorControl2(self.bot, self.head,p.POSITION_CONTROL, targetPosition = currentPos[0]-(i/100))
				p.stepSimulation()
				time.sleep(1./240.)
				i = i+0.01
		p.setJointMotorControl2(self.bot, self.head,p.VELOCITY_CONTROL, targetVelocity = 0)

	#function to close the gripper
	def close_gripper(self, size, count=0):
		i = 0
		currentPos_init = p.getJointState(self.bot, self.plate_left)
		currentPos = p.getJointState(self.bot, self.plate_left)
		while(currentPos[0]<size):
			currentPos = p.getJointState(self.bot, self.plate_left)
			p.setJointMotorControl2(self.bot, self.plate_left,p.POSITION_CONTROL, targetPosition = currentPos[0]+(i/100))
			p.stepSimulation()
			time.sleep(1./240.)
			i = i+0.001
			#if loop runs for more than 600 times
			#it means that the object is a bit bigger
			#and it won't go to the desired width
			#so exit the loop
			if i>0.6:
				break
		#for suction count = 1
		#if gripper is used, maintain a constant velocity and force
		#otherwise set TargetVelocity to 0
		if count==0:
			p.setJointMotorControl2(self.bot, self.plate_left,p.VELOCITY_CONTROL, targetVelocity = 0.2, force = 10)
		else:
			p.setJointMotorControl2(self.bot, self.plate_left,p.VELOCITY_CONTROL, targetVelocity = 0, force = 10)
		#p.setJointMotorControl2(self.bot, self.plate_right,p.VELOCITY_CONTROL, targetVelocity = -0.09, force = 2)

	#function to open the gripper
	def open_gripper(self):
		i = 0
		p.setJointMotorControl2(self.bot, self.plate_left,p.VELOCITY_CONTROL, targetVelocity = 0)
		currentPos_init = p.getJointState(self.bot, self.plate_left)
		currentPos = p.getJointState(self.bot, self.plate_left)
		while(currentPos[0]>0):
			currentPos = p.getJointState(self.bot, self.plate_left)
			p.setJointMotorControl2(self.bot, self.plate_left,p.POSITION_CONTROL, targetPosition = currentPos[0]-(i/100))
			p.stepSimulation()
			time.sleep(1./240.)
			i = i+0.001
		p.setJointMotorControl2(self.bot, self.plate_left,p.VELOCITY_CONTROL, targetVelocity = 0)

	#function to move the frame -- not used in sim
	def move_frame(self, pos):
		init, _ = p.getBasePositionAndOrientation(self.bot)
		j = 0.15
		j3 = p.getJointState(self.bot,31)
		j5 = p.getJointState(self.bot,33)
		j7 = p.getJointState(self.bot,35)
		j9 = p.getJointState(self.bot,37)
		j14 = p.getJointState(self.bot,49)
		j16 = p.getJointState(self.bot,51)
		j18 = p.getJointState(self.bot,53)
		j20 = p.getJointState(self.bot,55)
		if(init[1]>pos):
			while(pos<init[1]):
				p.setJointMotorControl2(self.bot, 31,p.POSITION_CONTROL, targetPosition = j3[0]-j)
				p.setJointMotorControl2(self.bot, 33,p.POSITION_CONTROL, targetPosition = j5[0]+j)
				p.setJointMotorControl2(self.bot, 35,p.POSITION_CONTROL, targetPosition = j7[0]+j)
				p.setJointMotorControl2(self.bot, 37,p.POSITION_CONTROL, targetPosition = j9[0]-j)
				p.setJointMotorControl2(self.bot, 49,p.POSITION_CONTROL, targetPosition = j14[0]+j)
				p.setJointMotorControl2(self.bot, 51,p.POSITION_CONTROL, targetPosition = j16[0]-j)
				p.setJointMotorControl2(self.bot, 53,p.POSITION_CONTROL, targetPosition = j18[0]+j)
				p.setJointMotorControl2(self.bot, 55,p.POSITION_CONTROL, targetPosition = j20[0]-j)
				init, _ = p.getBasePositionAndOrientation(self.bot)
				j3 = p.getJointState(self.bot,31)
				j5 = p.getJointState(self.bot,33)
				j7 = p.getJointState(self.bot,35)
				j9 = p.getJointState(self.bot,37)
				j14 = p.getJointState(self.bot,49)
				j16 = p.getJointState(self.bot,51)
				j18 = p.getJointState(self.bot,53)
				j20 = p.getJointState(self.bot,55)
				p.stepSimulation()
				time.sleep(1./240.)
			k = 0
			while(k<100):
				p.resetBaseVelocity(self.bot,[0,0,0])
				p.stepSimulation()
				time.sleep(1./240.)
				k = k+1
			return None
		if(init[1]<pos):
			while(pos>init[1]):
				p.setJointMotorControl2(self.bot, 31,p.POSITION_CONTROL, targetPosition = j3[0]+j)
				p.setJointMotorControl2(self.bot, 33,p.POSITION_CONTROL, targetPosition = j5[0]-j)
				p.setJointMotorControl2(self.bot, 35,p.POSITION_CONTROL, targetPosition = j7[0]-j)
				p.setJointMotorControl2(self.bot, 37,p.POSITION_CONTROL, targetPosition = j9[0]+j)
				p.setJointMotorControl2(self.bot, 49,p.POSITION_CONTROL, targetPosition = j14[0]-j)
				p.setJointMotorControl2(self.bot, 51,p.POSITION_CONTROL, targetPosition = j16[0]+j)
				p.setJointMotorControl2(self.bot, 53,p.POSITION_CONTROL, targetPosition = j18[0]-j)
				p.setJointMotorControl2(self.bot, 55,p.POSITION_CONTROL, targetPosition = j20[0]+j)
				init, _ = p.getBasePositionAndOrientation(self.bot)
				j3 = p.getJointState(self.bot,31)
				j5 = p.getJointState(self.bot,33)
				j7 = p.getJointState(self.bot,35)
				j9 = p.getJointState(self.bot,37)
				j14 = p.getJointState(self.bot,49)
				j16 = p.getJointState(self.bot,51)
				j18 = p.getJointState(self.bot,53)
				j20 = p.getJointState(self.bot,55)
				p.stepSimulation()
				time.sleep(1./240.)	
			k = 0
			while(k<100):
				p.resetBaseVelocity(self.bot,[0,0,0])
				p.stepSimulation()
				time.sleep(1./240.)
				k = k+1
			return None
	
	#function to move frame and head together using PID control
	def move_frame_and_head(self, target_pos_frame ,target_pos_head):
		target_pos_head = -target_pos_head
		_count = 0

        # PID parameters
		kp=3
		kd=10
		ki=0.005
		total_error=0
		last_error = 0

		# Initializing PLOTTING LISTs:
		target_list_x = []
		actual_list_x = []

		while True:
			_count += 1
			(_, current_pos_frame, _), _ = p.getBasePositionAndOrientation(self.bot)
			(current_pos_head, _, _, _) = p.getJointState(self.bot, self.head)
			# If inside a threshold
			if abs(target_pos_frame - current_pos_frame) <= 0.01 and abs(target_pos_head - current_pos_head) < 0.01:
				break
			#PID control frame
			error = current_pos_frame - target_pos_frame
			last_error = error if _count == 1 else last_error
			target_velocity = kp*error + kd*(error-last_error) + ki*total_error
			last_error = error
			total_error += error
			# Clipping velocity to prevent frame toppling
			target_velocity = np.clip(target_velocity, -8, 8) 
			p.setJointMotorControl2(self.bot, 46,p.VELOCITY_CONTROL, targetVelocity = target_velocity)
			p.setJointMotorControl2(self.bot, 49,p.VELOCITY_CONTROL, targetVelocity = -target_velocity)
			p.setJointMotorControl2(self.bot, 52,p.VELOCITY_CONTROL, targetVelocity = target_velocity)
			p.setJointMotorControl2(self.bot, 55,p.VELOCITY_CONTROL, targetVelocity = -target_velocity)

			p.setJointMotorControl2(self.bot, 81,p.VELOCITY_CONTROL, targetVelocity = -target_velocity)
			p.setJointMotorControl2(self.bot, 84,p.VELOCITY_CONTROL, targetVelocity = +target_velocity)
			p.setJointMotorControl2(self.bot, 87,p.VELOCITY_CONTROL, targetVelocity = +target_velocity)
			p.setJointMotorControl2(self.bot, 90,p.VELOCITY_CONTROL, targetVelocity = -target_velocity)

			#if head is in between 1 cm of the required pos set targetPosition equal to current Position
			p.setJointMotorControl2(self.bot, self.head,p.VELOCITY_CONTROL, targetVelocity = target_pos_head - current_pos_head)
			p.stepSimulation()
			
			# Accumulating values for plotting:
			actual_list_x.append(target_pos_frame)
			target_list_x.append(current_pos_frame)

		# PLOT VALUES
		plt.figure(figsize=[12, 9])
		plt.subplot(1, 1, 1)
		plt.title('PID plot')
		plt.xlabel('Steps:')
		plt.ylabel('Values: ')
		plt.plot(target_list_x, color='green',  label='target_list_x')
		plt.plot(actual_list_x, color='red',  label='actual_list_x')
		plt.grid()
		plt.legend()

		# plt.show()
		plt.savefig('plot.png')
		plt.close()

		# FUNCTION TO STOP.
		pos, orn = p.getBasePositionAndOrientation(self.bot)	
		for _ in range (100):
			p.setJointMotorControl2(self.bot, 46,p.VELOCITY_CONTROL, targetVelocity =0)
			p.setJointMotorControl2(self.bot, 49,p.VELOCITY_CONTROL, targetVelocity =0)
			p.setJointMotorControl2(self.bot, 52,p.VELOCITY_CONTROL, targetVelocity =0)
			p.setJointMotorControl2(self.bot, 55,p.VELOCITY_CONTROL, targetVelocity =0)
			p.setJointMotorControl2(self.bot, 81,p.VELOCITY_CONTROL, targetVelocity =0)
			p.setJointMotorControl2(self.bot, 84,p.VELOCITY_CONTROL, targetVelocity =0)
			p.setJointMotorControl2(self.bot, 87,p.VELOCITY_CONTROL, targetVelocity =0)
			p.setJointMotorControl2(self.bot, 90,p.VELOCITY_CONTROL, targetVelocity =0)
			p.setJointMotorControl2(self.bot, self.head, p.VELOCITY_CONTROL, targetVelocity =0)
			p.resetBasePositionAndOrientation(self.bot,pos,orn)
			p.stepSimulation()
			time.sleep(1./240.)
		return None
	
	#function to rotate gripper
	def rotate_gripper(self, angle):
		info = p.getJointState(self.bot,self.servo)
		if(angle>0):
			while(info[0]<angle):
				p.setJointMotorControl2(self.bot, self.servo,p.VELOCITY_CONTROL, targetVelocity = 1.5, force = 0.15)
				info = p.getJointState(self.bot,self.servo)
				p.stepSimulation()
				time.sleep(1./240.)
			p.setJointMotorControl2(self.bot, self.servo,p.VELOCITY_CONTROL, targetVelocity = 0, force = 0.09)
			return None
		if(angle<0):
			while(info[0]>angle):
				p.setJointMotorControl2(self.bot, self.servo,p.VELOCITY_CONTROL, targetVelocity = -1.5, force = 0.15)
				info = p.getJointState(self.bot,self.servo)
				p.stepSimulation()
				time.sleep(1./240.)
			p.setJointMotorControl2(self.bot, self.servo,p.VELOCITY_CONTROL, targetVelocity = 0, force = 0.09)
			return None
		
	#function to reset gripper
	def reset_gripper(self):
		info = p.getJointState(self.bot,self.servo)
		if info[0]>0:
			while(info[0]>0):
				p.setJointMotorControl2(self.bot, self.servo,p.VELOCITY_CONTROL, targetVelocity = -0.8, force = 0.09)
				info = p.getJointState(self.bot,self.servo)
				p.stepSimulation()
				time.sleep(1./240.)
		else:
			while(info[0]<0):
				p.setJointMotorControl2(self.bot, self.servo,p.VELOCITY_CONTROL, targetVelocity = 0.8, force = 0.09)
				info = p.getJointState(self.bot,self.servo)
				p.stepSimulation()
				time.sleep(1./240.)
		p.setJointMotorControl2(self.bot, self.servo,p.VELOCITY_CONTROL, targetVelocity = 0, force = 0.09)
				
	#function to rotate camera
	def rotate_camera(self, angle):
		info = p.getJointState(self.bot,self.camera)
		while(info[0]<angle):
			p.setJointMotorControl2(self.bot, self.camera,p.VELOCITY_CONTROL, targetVelocity = -0.1, force = 0.09)
			info = p.getJointState(self.bot,self.camera)
			p.stepSimulation()
			time.sleep(1./240.)
		p.setJointMotorControl2(self.bot, self.camera,p.VELOCITY_CONTROL, targetVelocity = 0, force = 0.09)
	
	#function to reset camera
	def reset_camera(self):
		info = p.getJointState(self.bot,self.camera)
		while(info[0]>0):
			p.setJointMotorControl2(self.bot, self.camera,p.VELOCITY_CONTROL, targetVelocity = 0.1, force = 0.09)
			info = p.getJointState(self.bot,self.camera)
			p.stepSimulation()
			time.sleep(1./240.)
		p.setJointMotorControl2(self.bot, self.camera,p.VELOCITY_CONTROL, targetVelocity = 0, force = 0.09)
	
	#function to return the endeffector coordinates 
	def end_effector(self):
		return p.getLinkState(self.bot,self.end_effect)
	
	#function to extend the suction cup
	def suction_down(self, down=0.09):
		i = 0.005
		currentPos_init = p.getJointState(self.bot, self.suction)
		currentPos = p.getJointState(self.bot, self.suction)
		while(currentPos[0]>-down):
			currentPos = p.getJointState(self.bot, self.suction)
			p.setJointMotorControl2(self.bot, self.suction,p.POSITION_CONTROL, targetPosition = currentPos[0]-i)
			p.stepSimulation()
			time.sleep(1./240.)
		p.setJointMotorControl2(self.bot, self.suction,p.VELOCITY_CONTROL, targetVelocity = 0)

	#function to contract the suction cup
	def suction_up(self):
		i = 0.005
		currentPos_init = p.getJointState(self.bot, self.suction)
		currentPos = p.getJointState(self.bot, self.suction)
		step = 0
		while(currentPos[0]<0):
			currentPos = p.getJointState(self.bot, self.suction)
			p.setJointMotorControl2(self.bot, self.suction,p.POSITION_CONTROL, targetPosition = currentPos[0]+i)
			p.stepSimulation()
			time.sleep(1./240.)
			step+=1
			print(step)
			if step>1000:
				break
		p.setJointMotorControl2(self.bot, self.suction,p.VELOCITY_CONTROL, targetVelocity = 0)
	
	#function to simulation suction force
	def suction_force(self, object):
		pos_cup=p.getLinkState(self.bot,self.suction_cup)[0]
		orn_cup=p.getLinkState(self.bot,self.suction_cup)[1]
		pos_obj,orn_obj=p.getBasePositionAndOrientation(object)
		import pdb
		
		euler_orn=p.getEulerFromQuaternion(orn_cup)
		force_constant=20

		p.addUserDebugLine([pos_cup[0],pos_cup[1],pos_cup[2]],[pos_obj[0],pos_obj[1],pos_obj[2]],[0.0,1.0,0.])
		vector_cup2obj=[force_constant*(pos_cup[0]-pos_obj[0]),force_constant*(pos_cup[1]-pos_obj[1]),force_constant*(pos_cup[2]-pos_obj[2])]
		
		for _ in range(250):
			# p.applyExternalForce(object,-1,[euler_orn[0],euler_orn[1],euler_orn[2]+2.5],[pos_cup[0],pos_cup[1],pos_cup[2]],p.WORLD_FRAME)
			p.applyExternalForce(object,-1,vector_cup2obj,[pos_cup[0],pos_cup[1],pos_cup[2]],p.WORLD_FRAME)
			p.stepSimulation()
			print(vector_cup2obj,'vector_cup2obj')
			pos_obj,orn_obj=p.getBasePositionAndOrientation(object)
			pos_cup=p.getLinkState(self.bot,self.suction_cup)[0]
			vector_cup2obj=[force_constant*(pos_cup[0]-pos_obj[0]),force_constant*(pos_cup[1]-pos_obj[1]),force_constant*(pos_cup[2]-pos_obj[2])]
			time.sleep(1.0/240.0)
			if pos_obj[2]>pos_cup[2]-0.01:
				break
		cons=p.createConstraint(object,-1,self.bot,self.suction_cup,p.JOINT_FIXED,[0,0,1],[0,0,0],[pos_obj[0]-pos_cup[0],pos_obj[1]-pos_cup[1],pos_obj[2]-pos_cup[2]],orn_obj)
		for i in range(75):
			info = p.getJointState(self.bot,self.servo)
			p.setJointMotorControl2(self.bot, self.servo, p.POSITION_CONTROL, targetPosition = 0)
			p.stepSimulation()
		self.reset_gripper()
		return cons

		
	#function to remove suction force
	def remove_suction_force(self, cons):
		p.removeConstraint(cons)
		for i in range(10):
			p.stepSimulation()

	#function to move suction cup towards the object
	def move_suction_cup(self, target_pos_frame, target_pos_head):
		self.move_frame_and_head(target_pos_frame+0.06, target_pos_head-0.19)

	#function to get the list of random objects -- not used in sim
	def _get_random_object(self):
		selected_objects_filenames = []
		numbers = [501, 502, 504, 505, 506, 507, 509, 510]
		for i in range(self._numObjects):
			urdf_no = str(random.choice(numbers))
			selected_objects_filenames.append('random_urdfs/'+urdf_no+'/'+urdf_no+'.urdf')
		return selected_objects_filenames

	#function to get the list of objects used in sim
	def get_objects(self):
		selected_objects_filenames = ['random_urdfs/008/008.urdf',
									  'random_urdfs/934/934.urdf',
									  'random_urdfs/507/507.urdf',
									  'random_urdfs/622/622.urdf',
									  'random_urdfs/502/502.urdf',
									  
									  'random_urdfs/002/002.urdf',
									  'random_urdfs/000/000.urdf',
									  'cube_small.urdf',
									  'random_urdfs/184/184.urdf',
									  'random_urdfs/173/173.urdf',

									  'random_urdfs/505/505.urdf',
									  'random_urdfs/767/767.urdf',
									  'random_urdfs/018/018.urdf',
									  'random_urdfs/504/504.urdf',
									  'random_urdfs/996/996.urdf',

									  'sphere_small.urdf',
									  'random_urdfs/001/001.urdf',
									  'teddy_vhacd.urdf',
									  'jenga/jenga.urdf',
									  'random_urdfs/330/330.urdf',
									  					
									  'random_urdfs/459/459.urdf',
									  'random_urdfs/506/506.urdf',
									  'random_urdfs/503/503.urdf',
									  'random_urdfs/008/008.urdf',
									  'duck_vhacd.urdf']

		return selected_objects_filenames

	#function to place the objects in the arena
	def _place_objects(self, urdfList):
		objectUids = []
		xpos = -0.8
		ypos = -0.45
		count = 0
		count2 = 0
		x = 0
		y = 0 
		for urdf_name in urdfList:
			angle = np.pi / 2 + self._blockRandom * np.pi * random.random()
			orn = p.getQuaternionFromEuler([0,0,angle])
			urdf_path=os.path.join(self._urdfRoot, urdf_name)	
			uid = p.loadURDF(urdf_path, [xpos, ypos, 1], [orn[0], orn[1], orn[2], orn[3]])
			objectUids.append(uid)
			count+=1
			ypos-=0.4
			if count%5==0:
				xpos+=0.4
				ypos=-0.45
		return objectUids

	#function to get overhead cam image from pick and drop area
	def overhead_camera(self, pick_or_drop = 0):
		if pick_or_drop == 0:
			look_p = [0, -1.23, 1]
			cameraeyepos_p = [0, -1.23, 1.5]
			cameraup_p = [0, -1, 0]
			self._view_matrix_p = p.computeViewMatrix(cameraeyepos_p, look_p, cameraup_p)
			fov = 120
			aspect = self._width / self._height
			near = 0.01
			far = 0.6
			self._proj_matrix_p = p.computeProjectionMatrixFOV(fov, aspect, near, far)
			img_arr = p.getCameraImage(width=self._width,
                               height=self._height,
                               viewMatrix=self._view_matrix_p,
                               projectionMatrix=self._proj_matrix_p,
                               renderer=p.ER_BULLET_HARDWARE_OPENGL)
			rgb = img_arr[2]
			np_img_arr = np.reshape(rgb, (self._height, self._width, 4))
			return np_img_arr
		else:
			look_d = [0, 1.25, 0.05]
			cameraeyepos_d = [0, 1.25, 1.5]
			cameraup_d = [0, 1, 0]
			self._view_matrix_d = p.computeViewMatrix(cameraeyepos_d, look_d, cameraup_d)
			fov = 75
			aspect = self._width / self._height
			near = 0.01
			far = 1.5
			self._proj_matrix_d = p.computeProjectionMatrixFOV(fov, aspect, near, far)
			img_arr = p.getCameraImage(width=self._width,
                               height=self._height,
                               viewMatrix=self._view_matrix_d,
                               projectionMatrix=self._proj_matrix_d,
                               renderer=p.ER_BULLET_HARDWARE_OPENGL)
			rgb = img_arr[2]
			np_img_arr = np.reshape(rgb, (self._height, self._width, 4))
			return np_img_arr

	#function to get RGB-D images of objects
	def rgbd_images(self, xpos, ypos, zpos):
			width = 224
			height = 224
			look = [xpos, ypos, 1]
			cameraeyepos = [xpos, ypos, zpos-0.07]
			cameraup = [0, -1, 0]
			self._view_matrix = p.computeViewMatrix(cameraeyepos, look, cameraup)
			fov = 55
			aspect = width / height
			near = 0.01
			far = 0.6
			self._proj_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
			img_arr = p.getCameraImage(width=width,
                               height=height,
                               viewMatrix=self._view_matrix,
                               projectionMatrix=self._proj_matrix,
                               renderer=p.ER_BULLET_HARDWARE_OPENGL)
			rgb = img_arr[2]
			np_img_arr = np.reshape(rgb, (height, width, 4))
			np_img_arr = cv2.cvtColor(np_img_arr,cv2.COLOR_BGR2RGB)
			depth = img_arr[3]
			depth = np.reshape(depth, (height, width))
			top_surf = np.amin(depth)
			depth_image = np.zeros(np_img_arr.shape, dtype=np.float64)
			depth_image[:,:,0] = depth
			lower_bound=np.amin(depth_image[:,:,0])
			upper_bound=np.amax(depth_image[:,:,0])
			depth_image_new=np.zeros(np_img_arr.shape[:2], dtype=np.float64)
			depth_image_new[:,:] = depth
			depth_image_new[:,:] = depth_image_new[:,:]*(255/(upper_bound-lower_bound))-((255*lower_bound)/(upper_bound-lower_bound))
			cv2.imwrite(r"./src/CapturedImg/depth"+".png",depth_image_new)
			cv2.imwrite(r"./src/CapturedImg/color"+".png",np_img_arr)
			return top_surf