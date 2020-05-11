import os
import math
import gym
from gym import spaces
from gym.utils import seeding
import numpy as np
import time
import pybullet as pb
import random
import pybullet_data
import time
import cv2

"""
renders not working properly in direct mode. 
is_discrete is used for discrete action space.
absmov is used for absolte movement in coordinate space, if set to false the motion along each axis is only by 0.01 from present state.
if not flags are set during init, the env will work as a continuous action space absolute movement env.
"""


class kukaPouring(gym.Env):
    metadata = {'render.modes': ['human', 'rgb_array'], 'video.frames_per_second': 50}

    def __init__(self, renders= False, random_ops = False, path_planning = False, is_discrete = False, absmov = True, manual_cntrl = True):
        
        self._observation = []
        self._envStepCounter = 0
        self._renders = renders
        self._manual_cntrl = manual_cntrl
        self._path_planning = path_planning
        self._width = 128
        self._height = 128
        self.terminated = 0
        self._isDiscrete = is_discrete
        self._absmovement = absmov
        self.max_steps = 500

        self.table, self.robotid, self.glass, self.plane = None, None, None, None

        self.viewMat = pb.computeViewMatrix(cameraEyePosition=[0.1, 0.5, 2.0],cameraTargetPosition= [0.0, 0.5, 0.75], cameraUpVector=[0,1.0,0])
        self.projMatrix = pb.computeProjectionMatrixFOV(fov=45.0, aspect=1.0, nearVal=0.5, farVal=2.0)
        self._robot_state = []*4
        self._eeAng = 0.0
        if self._renders:
            cid = pb.connect(pb.SHARED_MEMORY)
            if (cid < 0):
                pb.connect(pb.GUI)
        else:
            pb.connect(pb.DIRECT)        
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())

        pb.setInternalSimFlags(0)
        self.reset()
        
        if (self._isDiscrete):
            self.action_space = spaces.MultiDiscrete([3,3,3,3]) #lower bound is always 0, upper bound is specified but never used, discrete sampling within range [0,upper)
        else:
            action_dim = 4
            self._action_bound = 1
            action_high = np.array([self._action_bound] * action_dim)
            self.action_space = spaces.Box(-action_high, action_high, dtype=np.float32)
            self.observation_space = spaces.Box(low=0,
                                        high=255,
                                        shape=(self._height, self._width, 3),
                                        dtype=np.uint8)

    def reset(self):

        self.terminated = 0
        self._envStepCounter = 0
        pb.resetSimulation()
        pb.setGravity(0,0,-9.8)
        self.plane = pb.loadURDF("plane.urdf")
        path_data = '/home/aswin/Desktop/pouring_sim/pybullet_data1'
        self.table = pb.loadURDF(os.path.join(path_data, "table/table.urdf"), 0.0, 1.0, 0, 0.0, 0.0, 0.0, 1.0)

        self.robotid = pb.loadURDF("new_mod/urdf/new_mod.urdf",[0.0,0.2,0.1] ,useFixedBase= 1)
        self.glass = pb.loadURDF("glass2/urdf/glass.urdf", [-0.15, 0.6, 0.625],[0.0, 0.0, 0.0, 1.0], useFixedBase= 1)


        pos = [0.0, 0.5, 0.75]
        jointPoses = pb.calculateInverseKinematics(self.robotid, 6, pos)
        for i in range(7):
            pb.setJointMotorControl2(self.robotid, i, pb.POSITION_CONTROL, jointPoses[i])

        pb.setRealTimeSimulation(1)
        time.sleep(0.1)
        self.sphere_id = pb.loadURDF("pybullet_data1/sphere_1cm.urdf", [0.0, 0.65, 0.8])
        pb.changeDynamics(self.sphere_id, -1, linearDamping=0.0, angularDamping=0.1)
        self._observation = self.getObservation()
        self._robot_state = list(pb.getLinkState(self.robotid, 6)[0])
        self._eeAng = 0.0
        return np.array(self._observation)

    def __del__(self):
        pb.disconnect()

    def getObservation(self):
        
        _,_,img_arr,_,_ = pb.getCameraImage(width=self._width,
                               height=self._height,
                               viewMatrix= self.viewMat,
                               projectionMatrix= self.projMatrix,
                               renderer = pb.ER_TINY_RENDERER)
        
        self._observation = img_arr[:, :, :3]
        
        return np.array(self._observation)

    def step(self, action):
        
        self._robot_state = list(pb.getLinkState(self.robotid, 6)[0])

        if self._isDiscrete:
            dx= [-0.01, 0.0, 0.01][action[0]]
            dy= [-0.01, 0.0, 0.01][action[1]]
            dz= [-0.01, 0.0, 0.01][action[2]]
            da= [-2.7, 0.00, 2.7][action[3]]
            act = [dx, dy, dz]
            action_mod = []
            for a, s in zip(act, self._robot_state):
                action_mod.append(a+s)
            action_mod.append(da)
            self._eeAng = da
            self.move_robot(action_mod)


        else: 
            if self._absmovement:
                assert len(action) == 4, "incorrect action length, must be 4.."
                self.move_robot(self.scaled_action(action))

            else:
                action_mod = []
                for a, s in zip(action[:3], self._robot_state):
                    action_mod.append(a*0.01 +s)
                action_mod.append((action[3]*0.1)+self._eeAng)
                self._eeAng = action_mod[-1]
                self.move_robot(action_mod)

        time.sleep(0.5)
        reward, done = self._is_termination()

        return self.getObservation(), reward, done , {}

    def scaled_action(self, action):
        range_val =  [0.3, 0.2, 0.15, 2.9]
        start = [0.0, 0.5, 0.75, 0.0]
        action_mod = []
        action_mod.append(action[0]*range_val[0])
        for i in range(1,3):
            action_mod.append(start[i]+range_val[i]*(action[i]+1)/2)
        action_mod.append(action[3]*range_val[3])
        return action_mod

    def move_robot(self, action):
        
        assert len(action) == 4, "incorrect action length after processing..."
        jointPoses = pb.calculateInverseKinematics(self.robotid, 6, action[:3])
        for i in range(7):
            pb.setJointMotorControl2(self.robotid, i, pb.POSITION_CONTROL, jointPoses[i])
        pb.setJointMotorControl2(self.robotid,
                                6,
                                pb.POSITION_CONTROL,
                                targetPosition=action[3])

    def _is_termination(self):
        
        
        gnd_collision = True if (len(pb.getClosestPoints(self.sphere_id, self.table, 0.01)) +
                                                 len(pb.getClosestPoints(self.sphere_id, self.plane, 0.01)))> 0 else False
        glass_collision = True if (len(pb.getClosestPoints(self.sphere_id, self.glass, 0.01))+
                                                 len(pb.getClosestPoints(self.robotid , self.glass, 0.01)))>0 else False
        spherePos, _ = pb.getBasePositionAndOrientation(self.sphere_id)
        s = 0
        for i, j in zip(spherePos, (-0.15, 0.6,0.641)):
            s += (j-i)**2
        s = s**0.5

        capture = True if s< 0.015 else False
        done = False
        if (gnd_collision or glass_collision) and not capture:
            reward = -10
            done = True
        elif glass_collision and capture:
            reward = 15
            done = True
        else:
            reward = self._reward()
        return reward, done

    def _reward(self):
        
        goal = [-0.063, 0.500, 0.806]
        cur = pb.getLinkState(self.robotid, 6)[0]
        dist = 0
        for i,j in zip(goal,list(cur)):
            dist += (i-j)**2
        dist = dist**0.5
        return -50*dist

    def plan_rrt(self):
        pass
    
    def manual_control(self):
        paramIds = []
        if self._renders:
            paramIds.append(pb.addUserDebugParameter("x", -0.3, 0.3, 0.0 ))
            paramIds.append(pb.addUserDebugParameter("y", 0.5, 0.7, 0.6 ))
            paramIds.append(pb.addUserDebugParameter("z", 0.75, 0.9, 0.825 ))
            paramIds.append(pb.addUserDebugParameter("glass_ang", -2.9, 2.9, 0.0 ))
    
            temp = [0, 0, 0, 0, 0]
            pos = [0,0,0]
            pb.setRealTimeSimulation(1)

            while(1):  
                pb.getCameraImage(width=256,
                               height=256,
                               viewMatrix=self.viewMat,
                               projectionMatrix=self.projMatrix,
                               renderer = pb.ER_TINY_RENDERER)
                print("reward: ", self._is_termination())    
                targetPos = []
                for i in range(len(paramIds)):
                    c = paramIds[i]
                    targetPos.append(pb.readUserDebugParameter(c))
                if(targetPos != temp):
                    pos[0] = targetPos[0]
                    pos[1] = targetPos[1]
                    pos[2] = targetPos[2]
                    jointPoses = pb.calculateInverseKinematics(self.robotid, 6, pos)
                    for i in range(7):
                        pb.setJointMotorControl2(self.robotid, i, pb.POSITION_CONTROL, jointPoses[i])
                pb.setJointMotorControl2(self.robotid,
                                6,
                                pb.POSITION_CONTROL,
                                targetPosition=targetPos[3])
        else:
            print("No option of manual control")

if __name__ == "__main__":
    
    k = kukaPouring(renders= True)
    time.sleep(3)
    #k.manual_control()
    print(k.step([0, 0, 1, 1]))
    time.sleep(3)
