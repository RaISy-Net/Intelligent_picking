from pybullet_envs.bullet.kukaGymEnv import KukaGymEnv
import random
import os
from gym import spaces
import time
import pybullet as p
import kukas
import numpy as np
import pybullet_data
import pdb
import distutils.dir_util
import glob
from pkg_resources import parse_version
import gym
import cv2

class KukaDiverseObjectEnv(KukaGymEnv):
  """Class for Kuka environment with diverse objects.
  In each episode some objects are chosen from a set of 1000 diverse objects.
  These 1000 objects are split 90/10 into a train and test set.
  """

  def __init__(self,
               urdfRoot=pybullet_data.getDataPath(),
               actionRepeat=80,
               isEnableSelfCollision=True,
               renders=False,
               isDiscrete=False,
               maxSteps=8,
               dv=0.06,
               removeHeightHack=False,
               blockRandom=0.3,
               cameraRandom=0,
               width=512,
               height=512,
               numObjects=2,
               isTest=False):
    """Initializes the KukaDiverseObjectEnv.
    Args:
      urdfRoot: The diretory from which to load environment URDF's.
      actionRepeat: The number of simulation steps to apply for each action.
      isEnableSelfCollision: If true, enable self-collision.
      renders: If true, render the bullet GUI.
      isDiscrete: If true, the action space is discrete. If False, the
        action space is continuous.
      maxSteps: The maximum number of actions per episode.
      dv: The velocity along each dimension for each action.
      removeHeightHack: If false, there is a "height hack" where the gripper
        automatically moves down for each action. If true, the environment is
        harder and the policy chooses the height displacement.
      blockRandom: A float between 0 and 1 indicated block randomness. 0 is
        deterministic.
      cameraRandom: A float between 0 and 1 indicating camera placement
        randomness. 0 is deterministic.
      width: The image width.
      height: The observation image height.
      numObjects: The number of objects in the bin.
      isTest: If true, use the test set of objects. If false, use the train
        set of objects.
    """

    self._isDiscrete = isDiscrete
    self._timeStep = 1. / 240.
    self._urdfRoot = urdfRoot
    self._actionRepeat = actionRepeat
    self._isEnableSelfCollision = isEnableSelfCollision
    self._observation = []
    self._envStepCounter = 0
    self._renders = renders
    self._maxSteps = maxSteps
    self.terminated = 0
    self._cam_dist = 1.3
    self._cam_yaw = 180
    self._cam_pitch = -40
    self._dv = dv
    self._p = p
    self._removeHeightHack = removeHeightHack
    self._blockRandom = blockRandom
    self._cameraRandom = cameraRandom
    self._width = width
    self._height = height
    self._numObjects = numObjects
    self._isTest = isTest
    self._removal=[]
    self._alpha=1
   
    self.cid = p.connect(p.GUI)         #changed to always give OpenGL simulation
    #p.resetDebugVisualizerCamera(1.3, 180, -41, [0.52, -0.2, -0.33])  
    #p.resetDebugVisualizerCamera(0.35, -90, -90, [0.6, 0.1, -0.15])
    p.resetDebugVisualizerCamera(1.3 , 0, -41, [0, 0, 0.5])
    self.seed()

    if (self._isDiscrete):
      if self._removeHeightHack:
        self.action_space = spaces.Discrete(9)
      else:
        self.action_space = spaces.Discrete(7)
    else:
      self.action_space = spaces.Box(low=-1, high=1, shape=(3,))  # dx, dy, da
      if self._removeHeightHack:
        self.action_space = spaces.Box(low=-1, high=1, shape=(4,))  # dx, dy, dz, da
    self.viewer = None

  def reset(self):
    """Environment reset called at the beginning of an episode.
    """
    # Set the camera settings.
    look = [0, -0.5, 0.6]
    cameraeyepos = [0, -0.5, 0.7]
    cameraup = [0, -1, 0]
    self._view_matrix = p.computeViewMatrix(cameraeyepos, look, cameraup)
    fov = 70
    aspect = self._width / self._height
    near = 0.01
    far = 0.5
    self._proj_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)

    self._attempted_grasp = False
    self._env_step = 0
    self.terminated = 0

    p.resetSimulation()
    p.setPhysicsEngineParameter(numSolverIterations=150)
    p.setTimeStep(self._timeStep)
    p.loadURDF(os.path.join(self._urdfRoot, "plane.urdf"), [0, 0, -1])

    #p.loadURDF(os.path.join(self._urdfRoot, "table/table.urdf"), 0.5000000, 0.00000, -.820000,
               #0.000000, 0.000000, 0.0, 1.0)

    p.setGravity(0, 0, -10)
    self._kuka = kukas.Kuka(urdfRootPath=self._urdfRoot, timeStep=self._timeStep)
    self._envStepCounter = 0
    p.stepSimulation()

    # Choose the objects in the bin.
    #
    #self._get_random_object(self._numObjects,self._isTest)
    urdfList = self._get_random_object(self._numObjects,self._isTest)
    self._objectUids = self._place_objects(urdfList)
    self._observation = self._get_observation()
    return np.array(self._observation)

  def _randomly_place_objects(self, urdfList):
    """Randomly places the objects in the bin.
    Args:
      urdfList: The list of urdf files to place in the bin.
    Returns:
      The list of object unique ID's.
    """

    # Randomize positions of each object urdf.
    objectUids = []
    for urdf_name in urdfList:
      xpos = 0.2 * random.random() * self._alpha
      self._alpha*=-1
      ypos = -0.4 - 0.3 * (random.random())
      angle = np.pi / 2 + self._blockRandom * np.pi * random.random()
      orn = p.getQuaternionFromEuler([0, 0, angle])
      urdf_path = os.path.join(self._urdfRoot, urdf_name)
      uid = p.loadURDF(urdf_path, [xpos, ypos, 0.8], [orn[0], orn[1], orn[2], orn[3]])
      objectUids.append(uid)
      # Let each object fall to the tray individual, to prevent object
      # intersection.
      for _ in range(500):
        p.stepSimulation()
    return objectUids

  def _place_objects(self, urdfList):
      objectUids = []
      xpos = -0.1
      ypos = -0.5
      count = 0
      for urdf_name in urdfList:
          angle = np.pi / 2 + self._blockRandom * np.pi * random.random()
          orn = p.getQuaternionFromEuler([0,0,angle])
          urdf_path=os.path.join(self._urdfRoot, urdf_name)
          uid = p.loadURDF(urdf_path, [xpos, ypos, 0.5], [orn[0], orn[1], orn[2], orn[3]])
          objectUids.append(uid)
          for _ in range(500):
              p.stepSimulation()
          count+=1
          ypos-=0.17
          if count%1==0:
              xpos+=0.2
              ypos = -0.5
      return objectUids

  def _get_observation(self):
    """Return the observation as an image.
    """
    img_arr = p.getCameraImage(width=self._width,
                               height=self._height,
                               viewMatrix=self._view_matrix,
                               projectionMatrix=self._proj_matrix,
                               renderer=p.ER_BULLET_HARDWARE_OPENGL)
    rgb = img_arr[2]
    np_img_arr = np.reshape(rgb, (self._height, self._width, 4))
    #depth = img_arr[3]
    #print(depth)
    #cv2.imshow("Image",depth_img_arr)
    #cv2.waitKey(0)
    return np_img_arr

  def _get_individual_image(self, x, y, z, a):
      look = [x, y, 0.43]
      camera_eyepos = [x, y, 0.63]
      cameraup = [0, -1, 0]
      viewmatrix = p.computeViewMatrix(camera_eyepos, look, cameraup)
      fov = 38
      aspect = self._width / self._height
      near = 0.01
      far = 0.5
      proj_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
      img_arr = p.getCameraImage(width=self._width,
                               height=self._height,
                               viewMatrix=viewmatrix,
                               projectionMatrix=proj_matrix,
                               renderer=p.ER_BULLET_HARDWARE_OPENGL)
      rgb = img_arr[2]
      np_img_arr = np.reshape(rgb, (self._height, self._width, 4))
      np_img_arr = cv2.cvtColor(np_img_arr,cv2.COLOR_BGR2RGB)
      depth = img_arr[3]
      depth = np.reshape(depth, (self._height, self._width))
      #print(depth.shape)
      depth_image = np.zeros(np_img_arr.shape, dtype=np.float64)
      #print(depth_image.shape)
      depth_image[:,:,0] = depth
      #print(depth_image)
      cv2.imshow("depth",depth_image)
      #cv2.waitKey(0)
      lower_bound=np.amin(depth_image[:,:,0])
      upper_bound=np.amax(depth_image[:,:,0])
      depth_image_new=np.zeros(np_img_arr.shape[:2], dtype=np.float64)
      depth_image_new[:,:] = depth
      depth_image_new[:,:] = depth_image_new[:,:]*(255/(upper_bound-lower_bound))-((255*lower_bound)/(upper_bound-lower_bound))
      cv2.imwrite(r"C:/Users/yashs/OneDrive/Desktop/PS simulation/simulation_images/depth"+str(a)+".jpeg",depth_image_new)
      cv2.imwrite(r"C:/Users/yashs/OneDrive/Desktop/PS simulation/simulation_images/color"+str(a)+".jpeg",np_img_arr)
      

  def step(self, action):
    """Environment step.
    Args:
      action: 5-vector parameterizing XYZ offset, vertical angle offset
      (radians), and grasp angle (radians).
    Returns:
      observation: Next observation.
      reward: Float of the per-step reward as a result of taking the action.
      done: Bool of whether or not the episode has ended.
      debug: Dictionary of extra information provided by environment.
    """
    dv = self._dv  # velocity per physics step.
    if self._isDiscrete:
      # Static type assertion for integers.
      assert isinstance(action, int)
      if self._removeHeightHack:
        dx = [0, -dv, dv, 0, 0, 0, 0, 0, 0][action]
        dy = [0, 0, 0, -dv, dv, 0, 0, 0, 0][action]
        dz = [0, 0, 0, 0, 0, -dv, dv, 0, 0][action]
        da = [0, 0, 0, 0, 0, 0, 0, -0.25, 0.25][action]
      else:
        dx = [0, -dv, dv, 0, 0, 0, 0][action]
        dy = [0, 0, 0, -dv, dv, 0, 0][action]
        dz = -dv
        da = [0, 0, 0, 0, 0, -0.25, 0.25][action]
    else:
      dx = dv * action[0]
      dy = dv * action[1]
      if self._removeHeightHack:
        dz = dv * action[2]
        da = 0.25 * action[3]
      else:
        dz = -dv
        da = 0.25 * action[3]

    return self._step_continuous([dx, dy, dz, da, [0,0,0,0]])

  def _step_continuous(self, action):
    """Applies a continuous velocity-control action.
    Args:
      action: 5-vector parameterizing XYZ offset, vertical angle offset
      (radians), and grasp angle (radians).
    Returns:
      observation: Next observation.
      reward: Float of the per-step reward as a result of taking the action.
      done: Bool of whether or not the episode has ended.
      debug: Dictionary of extra information provided by environment.
    """
    # Perform commanded action.
    self._env_step += 1
    self._kuka.applyAction(action)
    for _ in range(self._actionRepeat):
      p.stepSimulation()
      time.sleep(0.00001)
      state = p.getLinkState(self._kuka.kukaUid, self._kuka.kukaEndEffectorIndex)
      end_effector_pos = state[0]
      if self._termination():
        break

    # If we are close to the bin, attempt grasp.
    state = p.getLinkState(self._kuka.kukaUid, self._kuka.kukaEndEffectorIndex)
    end_effector_pos = state[0]
    if end_effector_pos[2] <= 0.44:#0.1 for prev gripper
      print("attempting grasp")
      print(end_effector_pos)
      motor_fing = np.pi/4
      hinge_fing = -np.pi/6
      #grab  config
      grab = np.array([motor_fing,hinge_fing,-motor_fing,hinge_fing])
      #grab = np.array([0,0,0,0])
      for _ in range(500):
        grasp_action = [0, 0, 0, 0, grab]
        self._kuka.applyAction(grasp_action)
        p.stepSimulation()
        #if self._renders:
        #time.sleep(self._timeStep)
        #finger_angle -= 0.3 / 100.
        #if finger_angle < 0:
          #finger_angle = 0
      for _ in range(100000):
        grasp_action = [0, 0, 0.001, 0, grab]
        self._kuka.applyAction(grasp_action)
        p.stepSimulation()
        state = p.getLinkState(self._kuka.kukaUid, self._kuka.kukaEndEffectorIndex)
        end_effector_pos = state[0]
        if end_effector_pos[2]>0.8:
            break

        #time.sleep(self._timeStep)
        #finger_angle -= 0.3 / 100.
        #if finger_angle < 0:
          #finger_angle = 0
      

      #for _ in range(1000):
        #grasp_action = [5, 0, -0.001, 0, finger_angle]
        #self._kuka.applyAction(grasp_action)
        #p.stepSimulation()
        #time.sleep(self._timeStep)
        #finger_angle -= 0.3 / 100.
      #finger_angle=0.3
      #for _ in range(10000):
        #grasp_action = [-5, 0, 0.001, 0, finger_angle]
        #self._kuka.applyAction(grasp_action)
        #p.stepSimulation()
        #time.sleep(self._timeStep)
      self._attempted_grasp = True  
    observation = self._get_observation()
    done = self._termination()
    reward = self._reward()
    for body in self._removal:
        p.removeBody(body)
        self._objectUids.remove(body)
    self._removal=[]
    for _ in range(100):
          grasp_action=[0,0,0,0,np.zeros((4))]
          self._kuka.applyAction(grasp_action)
    
    debug = {'grasp_success': self._graspSuccess}
    return observation, reward, done, debug

  def _reward(self):
    """Calculates the reward for the episode.
    The reward is 1 if one of the objects is above height .2 at the end of the
    episode.
    """
    reward = 0
    self._graspSuccess = 0
    for uid in self._objectUids:
      pos, _ = p.getBasePositionAndOrientation(uid)
      # If any block is above height, provide reward.
      if pos[2] > 0.6:
        self._graspSuccess += 1
        reward = 1
        self._removal.append(uid)
        break
    return reward

  def _termination(self):
    """Terminates the episode if we have tried to grasp or if we are above
    maxSteps steps.
    """
    return self._attempted_grasp

  def _get_random_object(self, num_objects, test):
    """Randomly choose an object urdf from the random_urdfs directory.
    Args:
      num_objects:
        Number of graspable objects.
    Returns:
      A list of urdf filenames.
    """
    selected_objects_filenames = []
    numbers = [503, 510]
    for i in range(num_objects):
        #urdf_no = str(random.randint(500,510))
        urdf_no = str(numbers[i])
        selected_objects_filenames.append('random_urdfs/'+urdf_no+'/'+urdf_no+'.urdf')
    return selected_objects_filenames

  if parse_version(gym.__version__) < parse_version('0.9.6'):
    _reset = reset
    _step = step