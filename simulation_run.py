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

from src.robot import robot
from src.drop_area import *
from src.grasp_estimation import GraspEstimation
from src.robot_with_drop_area import GridEnvironment

if __name__=="__main__":
    simulation = GridEnvironment()
    simulation.do_simulation()
