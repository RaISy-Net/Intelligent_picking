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
from drop_area import *


class robot:
	def __init__(self, urdfRoot = pybullet_data.getDataPath(), num_Objects = 25, blockRandom = 0.3):
		p.connect(p.GUI)
		p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"), 0, 0, 0)
		self.bot = p.loadURDF('bot.urdf',basePosition = [0,0,2])
		self.rail1 = p.loadURDF('rail1.urdf',basePosition = [-1.25,0,0.2],baseOrientation = p.getQuaternionFromEuler([0,0,np.pi/2]))
		self.rail2 = p.loadURDF('rail1.urdf',basePosition = [1.25,0,0.2],baseOrientation = p.getQuaternionFromEuler([0,0,np.pi/2]))
		self.cam1 = p.loadURDF('cam1.urdf',basePosition = [1.5,-1,2],baseOrientation = p.getQuaternionFromEuler([0,0,np.pi/2]),useFixedBase = True)
		self.cam2 = p.loadURDF('cam1.urdf',basePosition = [1.5,1,2],baseOrientation = p.getQuaternionFromEuler([0,0,np.pi/2]),useFixedBase = True)
		p.setGravity(0,0,-10)
		self.n = p.getNumJoints(self.bot)
		self.wrist = 11
		self.mid_arm = 8
		self.upper_arm = 5
		self.head = 0
		self.gripper_plate = 16
		self.servo = 13
		self.camera = 19
		self.end_effect = 13
		self.suction = 24
		
		for i in range(self.n):
			print(i)
			print(p.getJointInfo(self.bot,i))

		for _ in range(500):
			p.stepSimulation()
		
		p.setPhysicsEngineParameter(numSolverIterations=150)

		self.Area_Halfdim = 1
		self._blockRandom = blockRandom
		self._urdfRoot = urdfRoot
		self._width = 1024
		self._height = 1024
		
		MakeArena(x=0,y=0,z=0.05,
	      scale_x=self.Area_Halfdim,scale_y=self.Area_Halfdim,scale_z=0,
	      Inter_area_dist=0.5,pickAreaHeight=0.90)
		
		self.overhead_camera(1)

		self._numObjects = num_Objects
		urdfList = self._get_random_object()
		self._objectUids = self._place_objects(urdfList)

		for _ in range(500):
			p.stepSimulation()
		print("done")
		img = self.overhead_camera(0)
		img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
		cv2.imwrite("C:/Users/yashs/OneDrive/Desktop/object detection images/image0.jpeg", img)

	def reset(self,a):
		p.resetSimulation()
		p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"), 0, 0, 0)
		self.bot = p.loadURDF('bot.urdf',basePosition = [0,0,2])
		self.rail1 = p.loadURDF('rail1.urdf',basePosition = [-1.25,0,0.2],baseOrientation = p.getQuaternionFromEuler([0,0,np.pi/2]))
		self.rail2 = p.loadURDF('rail1.urdf',basePosition = [1.25,0,0.2],baseOrientation = p.getQuaternionFromEuler([0,0,np.pi/2]))
		self.cam1 = p.loadURDF('cam1.urdf',basePosition = [1.5,-1,2],baseOrientation = p.getQuaternionFromEuler([0,0,np.pi/2]),useFixedBase = True)
		self.cam2 = p.loadURDF('cam1.urdf',basePosition = [1.5,1,2],baseOrientation = p.getQuaternionFromEuler([0,0,np.pi/2]),useFixedBase = True)
		p.setGravity(0,0,-10)
		self.n = p.getNumJoints(self.bot)

		MakeArena(x=0,y=0,z=0.05,
	      scale_x=self.Area_Halfdim,scale_y=self.Area_Halfdim,scale_z=0,
	      Inter_area_dist=0.5,pickAreaHeight=0.90)
		
		#self._numObjects = num_Objects
		urdfList = self._get_random_object()
		self._objectUids = self._place_objects(urdfList)
		for _ in range(500):
			p.stepSimulation()
		img = self.overhead_camera(0)
		img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
		cv2.imwrite("C:/Users/yashs/OneDrive/Desktop/object detection images/image"+str(a)+".jpeg", img)
	
	def extend_arm(self):
		i = 0
		j = 0
		k = 0
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
		
		
		
		

			

	def contract_arm(self):
		i = 0
		j = 0
		k = 0
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

				

	def close_gripper(self, size):
		i = 0
		currentPos_init = p.getJointState(self.bot, self.gripper_plate)
		currentPos = p.getJointState(self.bot, self.gripper_plate)
		while(currentPos[0]>currentPos_init[0] - size):
			currentPos = p.getJointState(self.bot, self.gripper_plate)
			p.setJointMotorControl2(self.bot, self.gripper_plate,p.POSITION_CONTROL, targetPosition = currentPos[0]-(i/100))
			p.stepSimulation()
			time.sleep(1./240.)
			i = i+0.001
			if i>0.7:
				print('not going further')
				break
		p.setJointMotorControl2(self.bot, self.gripper_plate,p.VELOCITY_CONTROL, targetVelocity = 0)


	def open_gripper(self):
		i = 0
		currentPos_init = p.getJointState(self.bot, self.gripper_plate)
		currentPos = p.getJointState(self.bot, self.gripper_plate)
		while(currentPos[0]<0):
			currentPos = p.getJointState(self.bot, self.gripper_plate)
			p.setJointMotorControl2(self.bot, self.gripper_plate,p.POSITION_CONTROL, targetPosition = currentPos[0]+(i/100))
			p.stepSimulation()
			time.sleep(1./240.)
			i = i+0.001
		p.setJointMotorControl2(self.bot, self.gripper_plate,p.VELOCITY_CONTROL, targetVelocity = 0)



	def move_frame(self, pos):
		init, ori = p.getBasePositionAndOrientation(self.bot)
		#distance = pos - init[1]
		#angle = distance/(2*np.pi*0.09)
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
				init, ori = p.getBasePositionAndOrientation(self.bot)
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
				init, ori = p.getBasePositionAndOrientation(self.bot)
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


	def move_frame_and_head(self, pos_frame ,pos_head):
		kp=3
		kd=0
		ki=0.005
		i=0
		t=0
		total_error=0
		pos_head = -pos_head
		init, ori = p.getBasePositionAndOrientation(self.bot)
		j = 0
		# p.setJointMotorControl2(self.bot, self.head,p.VELOCITY_CONTROL, targetVelocity = 0)
		if(1):
			last_error=0
			error=0
			counter=0

			while(1):
				error=init[1]-pos_frame
				total_error=total_error+error
				j=kp*error+kd*(error-last_error)+ki*total_error
				p.setJointMotorControl2(self.bot, 31,p.VELOCITY_CONTROL, targetVelocity = -j)
				p.setJointMotorControl2(self.bot, 33,p.VELOCITY_CONTROL, targetVelocity = j)
				p.setJointMotorControl2(self.bot, 35,p.VELOCITY_CONTROL, targetVelocity = j)
				p.setJointMotorControl2(self.bot, 37,p.VELOCITY_CONTROL, targetVelocity = -j)
				p.setJointMotorControl2(self.bot, 49,p.VELOCITY_CONTROL, targetVelocity = +j)
				p.setJointMotorControl2(self.bot, 51,p.VELOCITY_CONTROL, targetVelocity = -j)
				p.setJointMotorControl2(self.bot, 53,p.VELOCITY_CONTROL, targetVelocity = +j)
				p.setJointMotorControl2(self.bot, 55,p.VELOCITY_CONTROL, targetVelocity = -j)
				init, ori = p.getBasePositionAndOrientation(self.bot)
				# if init[1]>pos_frame-0.2 and init[1]<pos_frame+0.2:
				# 	kp=30/2
				increment=0.01
				
				currentPos = p.getJointState(self.bot, self.head)
				if currentPos[0]>pos_head:
					increment=-0.01
				if currentPos[0] < pos_head+0.01 and currentPos[0] > pos_head -0.01:
					i=0
					print("x mein ruk gaya")
				p.setJointMotorControl2(self.bot, self.head,p.POSITION_CONTROL, targetPosition = currentPos[0]+(i/100))
				i=i+increment
				p.stepSimulation()
				last_error=error
				if init[1]>pos_frame-0.01 and init[1]<pos_frame+0.01:
					counter+=1
				else:
					counter=0
				# print(init[1],'init')
				# print(pos_frame,'pos_frame')
				t=t+1
				print(t)
				if counter > 5:
					j=0
					print("y mein ruk gaya")

				if counter > 5 and currentPos[0] < pos_head+0.02 and currentPos[0] > pos_head -0.02:
					break
				
			k = 0
			while(k<100):
				p.setJointMotorControl2(self.bot, 31,p.VELOCITY_CONTROL, targetVelocity =0)
				p.setJointMotorControl2(self.bot, 33,p.VELOCITY_CONTROL, targetVelocity =0)
				p.setJointMotorControl2(self.bot, 35,p.VELOCITY_CONTROL, targetVelocity =0)
				p.setJointMotorControl2(self.bot, 37,p.VELOCITY_CONTROL, targetVelocity =0)
				p.setJointMotorControl2(self.bot, 49,p.VELOCITY_CONTROL, targetVelocity =0)
				p.setJointMotorControl2(self.bot, 51,p.VELOCITY_CONTROL, targetVelocity =0)
				p.setJointMotorControl2(self.bot, 53,p.VELOCITY_CONTROL, targetVelocity =0)
				p.setJointMotorControl2(self.bot, 55,p.VELOCITY_CONTROL, targetVelocity =0)
				p.stepSimulation()
				time.sleep(1./240.)
				k = k+1
			return None
			

	def rotate_gripper(self, angle):
		info = p.getJointState(self.bot,self.servo)
		if(angle>0):
			while(info[0]<angle):
				p.setJointMotorControl2(self.bot, self.servo,p.VELOCITY_CONTROL, targetVelocity = 0.8, force = 0.09)
				info = p.getJointState(self.bot,self.servo)
				p.stepSimulation()
				time.sleep(1./240.)
				print(info[0])
			p.setJointMotorControl2(self.bot, self.servo,p.VELOCITY_CONTROL, targetVelocity = 0, force = 0.09)
			return None
		if(angle<0):
			while(info[0]>angle):
				p.setJointMotorControl2(self.bot, self.servo,p.VELOCITY_CONTROL, targetVelocity = -0.8, force = 0.09)
				info = p.getJointState(self.bot,self.servo)
				p.stepSimulation()
				time.sleep(1./240.)
				print(info[0])
			p.setJointMotorControl2(self.bot, self.servo,p.VELOCITY_CONTROL, targetVelocity = 0, force = 0.09)
			return None
		
			
	def reset_gripper(self):
		info = p.getJointState(self.bot,self.servo)
		while(info[0]>0):
			p.setJointMotorControl2(self.bot, self.servo,p.VELOCITY_CONTROL, targetVelocity = -0.3, force = 0.09)
			info = p.getJointState(self.bot,self.servo)
			p.stepSimulation()
			time.sleep(1./240.)
		p.setJointMotorControl2(self.bot, self.servo,p.VELOCITY_CONTROL, targetVelocity = 0, force = 0.09)
				
				
	def rotate_camera(self, angle):
		info = p.getJointState(self.bot,self.camera)
		while(info[0]<angle):
			p.setJointMotorControl2(self.bot, self.camera,p.VELOCITY_CONTROL, targetVelocity = -0.1, force = 0.09)
			info = p.getJointState(self.bot,self.camera)
			p.stepSimulation()
			time.sleep(1./240.)
		p.setJointMotorControl2(self.bot, self.camera,p.VELOCITY_CONTROL, targetVelocity = 0, force = 0.09)
			
	def reset_camera(self):
		info = p.getJointState(self.bot,self.camera)
		while(info[0]>0):
			p.setJointMotorControl2(self.bot, self.camera,p.VELOCITY_CONTROL, targetVelocity = 0.1, force = 0.09)
			info = p.getJointState(self.bot,self.camera)
			p.stepSimulation()
			time.sleep(1./240.)
		p.setJointMotorControl2(self.bot, self.camera,p.VELOCITY_CONTROL, targetVelocity = 0, force = 0.09)
		
	def end_effector(self):
		return p.getLinkState(self.bot,self.end_effect)
		
		

	def suction_down(self):
		i = 0.001
		currentPos_init = p.getJointState(self.bot, self.suction)
		currentPos = p.getJointState(self.bot, self.suction)
		while(currentPos[0]>-0.09):
			currentPos = p.getJointState(self.bot, self.suction)
			p.setJointMotorControl2(self.bot, self.suction,p.POSITION_CONTROL, targetPosition = currentPos[0]-i)
			p.stepSimulation()
			time.sleep(1./240.)
		p.setJointMotorControl2(self.bot, self.suction,p.VELOCITY_CONTROL, targetVelocity = 0)

		

	def suction_up(self):
		i = 0.001
		currentPos_init = p.getJointState(self.bot, self.suction)
		currentPos = p.getJointState(self.bot, self.suction)
		while(currentPos[0]<0):
			currentPos = p.getJointState(self.bot, self.suction)
			p.setJointMotorControl2(self.bot, self.suction,p.POSITION_CONTROL, targetPosition = currentPos[0]+i)
			p.stepSimulation()
			time.sleep(1./240.)
		p.setJointMotorControl2(self.bot, self.suction,p.VELOCITY_CONTROL, targetVelocity = 0)


	def _get_random_object(self):
		"""Randomly choose an object urdf from the random_urdfs directory.
		Args:
		num_objects:
			Number of graspable objects.
		Returns:
		A list of urdf filenames.
		"""
		selected_objects_filenames = []
		#503,507,510
		numbers = [501, 502, 504, 505, 506, 507, 509, 510]
		for i in range(self._numObjects):
			urdf_no = str(random.choice(numbers))
			#urdf_no = str(numbers[i])
			selected_objects_filenames.append('random_urdfs/'+urdf_no+'/'+urdf_no+'.urdf')
		return selected_objects_filenames

	def _place_objects(self, urdfList):
		objectUids = []
		xpos = -0.8
		ypos = -0.45
		count = 0
		#xpos = random.uniform(-0.8, 0.8)
		#ypos = random.uniform(-0.45, -2.05)
		for urdf_name in urdfList:
			#xpos = random.uniform(-0.8, 0.8)
			#ypos = random.uniform(-0.45, -2.05)
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

	def rgbd_images(self, xpos, ypos, zpos):
			width = 224
			height = 224
			look = [xpos, ypos, 1]
			cameraeyepos = [xpos, ypos, zpos-0.07]
			print(zpos - 0.05)
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
			#print(depth.shape)
			depth_image = np.zeros(np_img_arr.shape, dtype=np.float64)
			#print(depth_image.shape)
			depth_image[:,:,0] = depth
			#print(depth_image)
			#cv2.imshow("depth",depth_image)
			#cv2.waitKey(0)
			lower_bound=np.amin(depth_image[:,:,0])
			upper_bound=np.amax(depth_image[:,:,0])
			depth_image_new=np.zeros(np_img_arr.shape[:2], dtype=np.float64)
			depth_image_new[:,:] = depth
			depth_image_new[:,:] = depth_image_new[:,:]*(255/(upper_bound-lower_bound))-((255*lower_bound)/(upper_bound-lower_bound))
			cv2.imwrite(r"C:/Users/yashs/OneDrive/Desktop/depth"+".png",depth_image_new)
			cv2.imwrite(r"C:/Users/yashs/OneDrive/Desktop/color"+".png",np_img_arr)
				
if __name__ == "__main__":
	bot = robot()
	while(True):
		bot.move_frame_and_head(0.8, 1)
		bot.suction_down()
		bot.suction_up()
		bot.move_frame(-1)
		print(bot.end_effector())
		bot.move_head(0.9)
		print(bot.end_effector())
		bot.extend_wrist(0.10)
		print(bot.end_effector())
		bot.close_gripper(0.06)
		print(bot.end_effector())
		bot.contract_wrist(0.10)
		bot.move_head(0)
		bot.move_frame(0)
		bot.extend_arm()
		print(bot.end_effector())
		bot.open_gripper()
		bot.rotate_gripper(1.5707)
		bot.reset_gripper()
		bot.rotate_camera(1)
		bot.reset_camera()
		while(True):
			p.stepSimulation()
			time.sleep(1./240.)