import math
import gym
from gym import spaces
from gym.utils import seeding
import numpy as np
import time
import pybullet as p
from . import racecar
import random

class RacecarZEDGymEnv(gym.Env):
  metadata = {
      'render.modes': ['human', 'rgb_array'],
      'video.frames_per_second' : 50
  }

  def __init__(self,
               urdfRoot="",
               actionRepeat=100,
               isEnableSelfCollision=True,
               renders=True):
    print("init")
    self._timeStep = 0.01
    self._urdfRoot = urdfRoot
    self._actionRepeat = actionRepeat
    self._isEnableSelfCollision = isEnableSelfCollision
    self._ballUniqueId = -1
    self._envStepCounter = 0
    self._renders = renders
    self._width = 100
    self._height = 10
    self._p = p
    if self._renders:
      p.connect(p.GUI)
    else:
      p.connect(p.DIRECT)
    self._seed()
    self.reset()
    observationDim = len(self.getExtendedObservation())
    #print("observationDim")
    #print(observationDim)
    
    observation_high = np.array([np.finfo(np.float32).max] * observationDim)    
    self.action_space = spaces.Discrete(6)
    self.observation_space = spaces.Box(low=0, high=255, shape=(self._height, self._width, 4))

    self.viewer = None

  def _reset(self):
    p.resetSimulation()
    #p.setPhysicsEngineParameter(numSolverIterations=300)
    p.setTimeStep(self._timeStep)
    #p.loadURDF("%splane.urdf" % self._urdfRoot)
    stadiumobjects = p.loadSDF("%sstadium.sdf" % self._urdfRoot)
    #move the stadium objects slightly above 0
    for i in stadiumobjects:
      pos,orn = p.getBasePositionAndOrientation(i)
      newpos = [pos[0],pos[1],pos[2]+0.1]
      p.resetBasePositionAndOrientation(i,newpos,orn)
    
    dist = 5 +2.*random.random()
    ang = 2.*3.1415925438*random.random()
    
    ballx = dist * math.sin(ang)
    bally = dist * math.cos(ang)
    ballz = 1
    
    self._ballUniqueId = p.loadURDF("sphere2red.urdf",[ballx,bally,ballz])
    p.setGravity(0,0,-10)
    self._racecar = racecar.Racecar(urdfRootPath=self._urdfRoot, timeStep=self._timeStep)
    self._envStepCounter = 0
    for i in range(100):
      p.stepSimulation()
    self._observation = self.getExtendedObservation()
    return np.array(self._observation)

  def __del__(self):
    p.disconnect()

  def _seed(self, seed=None):
    self.np_random, seed = seeding.np_random(seed)
    return [seed]

  def getExtendedObservation(self):
     carpos,carorn = p.getBasePositionAndOrientation(self._racecar.racecarUniqueId)
     carmat = p.getMatrixFromQuaternion(carorn)
     ballpos,ballorn = p.getBasePositionAndOrientation(self._ballUniqueId)      
     invCarPos,invCarOrn = p.invertTransform(carpos,carorn)
     ballPosInCar,ballOrnInCar = p.multiplyTransforms(invCarPos,invCarOrn,ballpos,ballorn)
     dist0 = 0.3
     dist1 = 1.
     eyePos = [carpos[0]+dist0*carmat[0],carpos[1]+dist0*carmat[3],carpos[2]+dist0*carmat[6]+0.3]
     targetPos = [carpos[0]+dist1*carmat[0],carpos[1]+dist1*carmat[3],carpos[2]+dist1*carmat[6]+0.3]      
     up = [carmat[2],carmat[5],carmat[8]]
     viewMat = p.computeViewMatrix(eyePos,targetPos,up)
     #viewMat = p.computeViewMatrixFromYawPitchRoll(carpos,1,0,0,0,2)
     #print("projectionMatrix:")
     #print(p.getDebugVisualizerCamera()[3])
     projMatrix = [0.7499999403953552, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0000200271606445, -1.0, 0.0, 0.0, -0.02000020071864128, 0.0]
     img_arr = p.getCameraImage(width=self._width,height=self._height,viewMatrix=viewMat,projectionMatrix=projMatrix)
     rgb=img_arr[2]
     np_img_arr = np.reshape(rgb, (self._height, self._width, 4))
     self._observation = np_img_arr
     return self._observation
     
  def _step(self, action):
    if (self._renders):
      basePos,orn = p.getBasePositionAndOrientation(self._racecar.racecarUniqueId)
      #p.resetDebugVisualizerCamera(1, 30, -40, basePos)
    
    fwd = [5,0,5,10,10,10]
    steerings = [-0.5,0,0.5,-0.3,0,0.3]
    forward = fwd[action]
    steer = steerings[action]
    realaction = [forward,steer]
    self._racecar.applyAction(realaction)
    for i in range(self._actionRepeat):
      p.stepSimulation()
      if self._renders:
        time.sleep(self._timeStep)
      self._observation = self.getExtendedObservation()
      
      if self._termination():
        break
      self._envStepCounter += 1
    reward = self._reward()
    done = self._termination()
    #print("len=%r" % len(self._observation))
    
    return np.array(self._observation), reward, done, {}

  def _render(self, mode='human', close=False):
      return

  def _termination(self):
    return self._envStepCounter>1000
    
  def _reward(self):
    closestPoints = p.getClosestPoints(self._racecar.racecarUniqueId,self._ballUniqueId,10000) 
    
    numPt = len(closestPoints)
    reward=-1000
    #print(numPt)
    if (numPt>0):
      #print("reward:")
      reward = -closestPoints[0][8]
      #print(reward)
    return reward
