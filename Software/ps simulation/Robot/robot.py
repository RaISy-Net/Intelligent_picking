
import pybullet as p
import pybullet_data
import os
import time

class robot:
	def __init__(self):
		p.connect(p.GUI)
		p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"), 0, 0, 0)
		self.bot = p.loadURDF('frame3.urdf',basePosition = [0,0,1.5])
		p.setGravity(0,0,-10)
		self.n = p.getNumJoints(self.bot)
		for i in range(self.n):
			print(i)
			print(p.getJointInfo(self.bot,i))
	
	def extend_arm(self):
		i = 0
		currentPos_init = p.getJointState(self.bot, 27)
		currentPos = p.getJointState(self.bot, 27)
		while(currentPos[0]>-0.28):
			currentPos = p.getJointState(self.bot, 27)
			p.setJointMotorControl2(self.bot, 27,p.POSITION_CONTROL, targetPosition = currentPos[0]-(i/100))
			p.stepSimulation()
			time.sleep(1./240.)
			i = i+0.001
		i = 0
		currentPos_init = p.getJointState(self.bot, 30)
		currentPos = p.getJointState(self.bot, 30)
		while(currentPos[0]>-0.28):
			currentPos = p.getJointState(self.bot, 30)
			p.setJointMotorControl2(self.bot, 30,p.POSITION_CONTROL, targetPosition = currentPos[0]-(i/100))
			p.stepSimulation()
			time.sleep(1./240.)
			i = i+0.001
		i = 0
		currentPos_init = p.getJointState(self.bot, 33)
		currentPos = p.getJointState(self.bot, 33)
		while(currentPos[0]>-0.28):
			currentPos = p.getJointState(self.bot, 33)
			p.setJointMotorControl2(self.bot, 33,p.POSITION_CONTROL, targetPosition = currentPos[0]-(i/100))
			p.stepSimulation()
			time.sleep(1./240.)
			i = i+0.001
			

	def contract_arm(self):
		i = 0
		currentPos_init = p.getJointState(self.bot, 33)
		currentPos = p.getJointState(self.bot, 33)
		while(currentPos[0]<0):
			currentPos = p.getJointState(self.bot, 33)
			p.setJointMotorControl2(self.bot, 33,p.POSITION_CONTROL, targetPosition = currentPos[0]+(i/100))
			p.stepSimulation()
			time.sleep(1./240.)
			i = i+0.001
		i = 0
		currentPos_init = p.getJointState(self.bot, 30)
		currentPos = p.getJointState(self.bot, 30)
		while(currentPos[0]<0):
			currentPos = p.getJointState(self.bot, 30)
			p.setJointMotorControl2(self.bot, 30,p.POSITION_CONTROL, targetPosition = currentPos[0]+(i/100))
			p.stepSimulation()
			time.sleep(1./240.)
			i = i+0.001
		i = 0
		currentPos_init = p.getJointState(self.bot, 27)
		currentPos = p.getJointState(self.bot, 27)
		while(currentPos[0]<0):
			currentPos = p.getJointState(self.bot, 27)
			p.setJointMotorControl2(self.bot, 27,p.POSITION_CONTROL, targetPosition = currentPos[0]+(i/100))
			p.stepSimulation()
			time.sleep(1./240.)
			i = i+0.001
		
	def extend_wrist(self, size):
		i = 0
		currentPos_init = p.getJointState(self.bot, 33)
		currentPos = p.getJointState(self.bot, 33)
		while(currentPos[0]>currentPos_init[0]-size):
			currentPos = p.getJointState(self.bot, 33)
			p.setJointMotorControl2(self.bot, 33,p.POSITION_CONTROL, targetPosition = currentPos[0]-(i/100))
			p.stepSimulation()
			time.sleep(1./240.)
			i = i+0.001

	def contract_wrist(self, size):
		i = 0
		currentPos_init = p.getJointState(self.bot, 33)
		currentPos = p.getJointState(self.bot, 33)
		while(currentPos[0]<currentPos_init[0] + size):
			currentPos = p.getJointState(self.bot, 33)
			p.setJointMotorControl2(self.bot, 33,p.POSITION_CONTROL, targetPosition = currentPos[0]+(i/100))
			p.stepSimulation()
			time.sleep(1./240.)
			i = i+0.001

	def move_head(self, pos):
		i = 0
		pos = -pos
		currentPos = p.getJointState(self.bot, 22)
		if(currentPos[0] < pos):
			while(currentPos[0]< pos):
				currentPos = p.getJointState(self.bot, 22)
				p.setJointMotorControl2(self.bot, 22,p.POSITION_CONTROL, targetPosition = currentPos[0]+(i/100))
				p.stepSimulation()
				time.sleep(1./240.)
				i = i+0.001
		if(currentPos[0] > pos):
			while(currentPos[0] > pos):
				currentPos = p.getJointState(self.bot, 22)
				p.setJointMotorControl2(self.bot, 22,p.POSITION_CONTROL, targetPosition = currentPos[0]-(i/100))
				p.stepSimulation()
				time.sleep(1./240.)
				i = i+0.001
				

	def close_gripper(self, size):
		i = 0
		currentPos_init = p.getJointState(self.bot, 38)
		currentPos = p.getJointState(self.bot, 38)
		while(currentPos[0]>currentPos_init[0] - size):
			currentPos = p.getJointState(self.bot, 38)
			p.setJointMotorControl2(self.bot, 38,p.POSITION_CONTROL, targetPosition = currentPos[0]-(i/100))
			p.stepSimulation()
			time.sleep(1./240.)
			i = i+0.001

	def open_gripper(self):
		i = 0
		currentPos_init = p.getJointState(self.bot, 38)
		currentPos = p.getJointState(self.bot, 38)
		while(currentPos[0]<0):
			currentPos = p.getJointState(self.bot, 38)
			p.setJointMotorControl2(self.bot, 38,p.POSITION_CONTROL, targetPosition = currentPos[0]+(i/100))
			p.stepSimulation()
			time.sleep(1./240.)
			i = i+0.001


	def move_frame(self, pos):
		init, ori = p.getBasePositionAndOrientation(self.bot)
		#distance = pos - init[1]
		#angle = distance/(2*np.pi*0.09)
		j = 0.05
		j3 = p.getJointState(self.bot,3)
		j5 = p.getJointState(self.bot,5)
		j7 = p.getJointState(self.bot,7)
		j9 = p.getJointState(self.bot,9)
		j14 = p.getJointState(self.bot,14)
		j16 = p.getJointState(self.bot,16)
		j18 = p.getJointState(self.bot,18)
		j20 = p.getJointState(self.bot,20)
		if(init[1]<pos):
			while(pos>init[1]):
				p.setJointMotorControl2(self.bot, 3,p.POSITION_CONTROL, targetPosition = j3[0]-j)
				p.setJointMotorControl2(self.bot, 5,p.POSITION_CONTROL, targetPosition = j5[0]+j)
				p.setJointMotorControl2(self.bot, 7,p.POSITION_CONTROL, targetPosition = j7[0]-j)
				p.setJointMotorControl2(self.bot, 9,p.POSITION_CONTROL, targetPosition = j9[0]+j)
				p.setJointMotorControl2(self.bot, 14,p.POSITION_CONTROL, targetPosition = j14[0]+j)
				p.setJointMotorControl2(self.bot, 16,p.POSITION_CONTROL, targetPosition = j16[0]-j)
				p.setJointMotorControl2(self.bot, 18,p.POSITION_CONTROL, targetPosition = j18[0]-j)
				p.setJointMotorControl2(self.bot, 20,p.POSITION_CONTROL, targetPosition = j20[0]+j)
				init, ori = p.getBasePositionAndOrientation(self.bot)
				j3 = p.getJointState(self.bot,3)
				j5 = p.getJointState(self.bot,5)
				j7 = p.getJointState(self.bot,7)
				j9 = p.getJointState(self.bot,9)
				j14 = p.getJointState(self.bot,14)
				j16 = p.getJointState(self.bot,16)
				j18 = p.getJointState(self.bot,18)
				j20 = p.getJointState(self.bot,20)
				p.stepSimulation()
				time.sleep(1./240.)
			k = 0
			while(k<100):
				p.resetBaseVelocity(self.bot,[0,0,0])
				p.stepSimulation()
				time.sleep(1./240.)
				k = k+1
			return None
		if(init[1]>pos):
			while(pos<init[1]):
				p.setJointMotorControl2(self.bot, 3,p.POSITION_CONTROL, targetPosition = j3[0]+j)
				p.setJointMotorControl2(self.bot, 5,p.POSITION_CONTROL, targetPosition = j5[0]-j)
				p.setJointMotorControl2(self.bot, 7,p.POSITION_CONTROL, targetPosition = j7[0]+j)
				p.setJointMotorControl2(self.bot, 9,p.POSITION_CONTROL, targetPosition = j9[0]-j)
				p.setJointMotorControl2(self.bot, 14,p.POSITION_CONTROL, targetPosition = j14[0]-j)
				p.setJointMotorControl2(self.bot, 16,p.POSITION_CONTROL, targetPosition = j16[0]+j)
				p.setJointMotorControl2(self.bot, 18,p.POSITION_CONTROL, targetPosition = j18[0]+j)
				p.setJointMotorControl2(self.bot, 20,p.POSITION_CONTROL, targetPosition = j20[0]-j)
				init, ori = p.getBasePositionAndOrientation(self.bot)
				j3 = p.getJointState(self.bot,3)
				j5 = p.getJointState(self.bot,5)
				j7 = p.getJointState(self.bot,7)
				j9 = p.getJointState(self.bot,9)
				j14 = p.getJointState(self.bot,14)
				j16 = p.getJointState(self.bot,16)
				j18 = p.getJointState(self.bot,18)
				j20 = p.getJointState(self.bot,20)
				p.stepSimulation()
				time.sleep(1./240.)	
			k = 0
			while(k<100):
				p.resetBaseVelocity(self.bot,[0,0,0])
				p.stepSimulation()
				time.sleep(1./240.)
				k = k+1
			return None



			

	def rotate_gripper(self, angle):
		info = p.getJointState(self.bot,35)
		while(info[0]<angle):
			p.setJointMotorControl2(self.bot, 35,p.VELOCITY_CONTROL, targetVelocity = 0.3, force = 0.09)
			info = p.getJointState(self.bot,35)
			p.stepSimulation()
			time.sleep(1./240.)
		p.setJointMotorControl2(self.bot, 35,p.VELOCITY_CONTROL, targetVelocity = 0, force = 0.09)
			
	def reset_gripper(self):
		info = p.getJointState(self.bot,35)
		while(info[0]>0):
			p.setJointMotorControl2(self.bot, 35,p.VELOCITY_CONTROL, targetVelocity = -0.3, force = 0.09)
			info = p.getJointState(self.bot,35)
			p.stepSimulation()
			time.sleep(1./240.)
		p.setJointMotorControl2(self.bot, 35,p.VELOCITY_CONTROL, targetVelocity = 0, force = 0.09)
				
				
	def rotate_camera(self, angle):
		info = p.getJointState(self.bot,41)
		while(info[0]<angle):
			p.setJointMotorControl2(self.bot, 41,p.VELOCITY_CONTROL, targetVelocity = -0.1, force = 0.09)
			info = p.getJointState(self.bot,41)
			p.stepSimulation()
			time.sleep(1./240.)
		p.setJointMotorControl2(self.bot, 41,p.VELOCITY_CONTROL, targetVelocity = 0, force = 0.09)
			
	def reset_camera(self):
		info = p.getJointState(self.bot,41)
		while(info[0]>0):
			p.setJointMotorControl2(self.bot, 41,p.VELOCITY_CONTROL, targetVelocity = 0.1, force = 0.09)
			info = p.getJointState(self.bot,41)
			p.stepSimulation()
			time.sleep(1./240.)
		p.setJointMotorControl2(self.bot, 41,p.VELOCITY_CONTROL, targetVelocity = 0, force = 0.09)
				
if __name__ == "__main__":
	bot = robot()
	while(True):
		bot.move_frame(1)
		bot.move_head(0.9)
		bot.extend_wrist(0.10)
		bot.close_gripper(0.06)
		bot.contract_wrist(0.10)
		bot.move_head(0)
		bot.move_frame(0)
		bot.extend_arm()
		bot.open_gripper()
		bot.rotate_gripper(1.5707)
		bot.reset_gripper()
		bot.rotate_camera(1)
		bot.reset_camera()
		while(True):
			p.stepSimulation()
			time.sleep(1./240.)

