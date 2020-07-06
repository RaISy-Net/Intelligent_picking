import pybullet as p
import time
import math
import numpy as np
from datetime import datetime
import pybullet_data
from drop_area import *

p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf", [0, 0, 0])

p.loadURDF("pick_drop_areas/traybox.urdf", [0, -4, 0])

# import_drop_area function arguments are x-x position of drop area,y-y position of drop area,
# z- z position of drop area, scale_x,scale_y,scale_z - scaling factors 
import_drop_area(1,1,0.5,scale_x=0.5,scale_y=0.5,scale_z=0.5)

# drop_area_tray=p.loadURDF("pick_drop_areas/traybox.urdf", [0, 0, 1.0])
grip_list= p.loadSDF("./kukka_wsg50/kuka_with_wsg50.sdf")
gripper1 = grip_list[0]
p.resetBasePositionAndOrientation(gripper1,[0,4,0],p.getQuaternionFromEuler([0,0,1.57]))
p.setGravity(0, 0, -10)



while 1:
  p.stepSimulation()
  time.sleep(1.0/100.0)
p.disconnect()
