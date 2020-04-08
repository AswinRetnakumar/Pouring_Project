import os
import pybullet as pb 
import time 
import math
import init_setup
import numpy as np

fingerAForce = 3
fingerBForce = 3
fingerTipForce = 2
kukaUid = None


def move_joint(kukaUid, jointang):

  
  for i in range(len(jointang)):
    pb.setJointMotorControl2(kukaUid, i, pb.POSITION_CONTROL, jointang[i])

def move_to_pose(kukaUid, pos, orn, wrist_ang = 0, orn_cntrl = False):


  if orn_cntrl :
    jointPoses = pb.calculateInverseKinematics(kukaUid, 7, pos, orn)
    for i in range(7):
        pb.setJointMotorControl2(kukaUid, i, pb.POSITION_CONTROL, jointPoses[i])
  
  else :
    jointPoses = pb.calculateInverseKinematics(kukaUid, 7, pos)
    for i in range(7):
      pb.setJointMotorControl2(kukaUid, i, pb.POSITION_CONTROL, jointPoses[i])
    pb.setJointMotorControl2(kukaUid,
                            6,
                            pb.POSITION_CONTROL,
                            targetPosition=wrist_ang)

def gripper_cntrl(kukaUid, fingerAngle):

  global fingerAForce
  global fingerBForce
  global fingerTipForce

  pb.setJointMotorControl2(kukaUid,
                                8,
                                pb.POSITION_CONTROL,
                                targetPosition=-fingerAngle,
                                force=fingerAForce)
  pb.setJointMotorControl2(kukaUid,
                                11,
                                pb.POSITION_CONTROL,
                                targetPosition=fingerAngle,
                                force=fingerBForce)

  pb.setJointMotorControl2(kukaUid,
                                10,
                                pb.POSITION_CONTROL,
                                targetPosition=0.4,
                                force=fingerTipForce)
  pb.setJointMotorControl2(kukaUid,
                                13,
                                pb.POSITION_CONTROL,
                                targetPosition=-0.4,
                                force=fingerTipForce)

def start(kukaUid):

  jointPositions = [
        1.6, 0.0 , 0.0, 1.642, 1.34, 1.0, -0.00, 0.0,
        0.0]
  orn = [1.0, 0.0, 0.0, 1.0]
  temp_pos = [-0.25, -0.4, 0.9]
  pos = [-0.1485, -0.418, 0.640]
  move_joint(kukaUid, jointPositions)
  time.sleep(.2)
  move_to_pose(kukaUid, temp_pos, orn, orn_cntrl= True)
  time.sleep(.2)
  move_to_pose(kukaUid, pos, orn, orn_cntrl= True)
  for gripping_ang in np.arange(start= 0.400, stop= 0.125, step= -0.005, dtype= np.float32):
    gripper_cntrl(kukaUid, gripping_ang)
    time.sleep(0.05)
  for z in np.arange(start= 0.640, stop= 0.750, step= 0.010, dtype= np.float32):
    pos[2] = z
    move_to_pose(kukaUid, pos, orn)
    time.sleep(0.01)
  '''Test wrist angle cntrl
  for ang in np.arange(start= -1.5, stop= 1.5, step= 0.10, dtype= np.float32):
    move_to_pose(kukaUid, pos, orn, ang)
    time.sleep(0.5)
  '''
def main():
   
  global kukaUid
  kukaUid = init_setup.load_data()
  start(kukaUid)

if __name__ == "__main__":
  main()
