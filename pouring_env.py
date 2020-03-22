import os
import pybullet as pb 
import time 
import math

path_data = '/home/aswin/Desktop/pouring_sim/pybullet_data1'
physicsClient = pb.connect(pb.GUI)
pb.setGravity(0,0,-9.8)
jointIds = []
paramIds = []

pb.setInternalSimFlags(0)

objects = pb.loadSDF(os.path.join(path_data, "kuka_iiwa/kuka_with_gripper.sdf"))
pb.loadURDF(os.path.join(path_data, "table/table.urdf"), 0.0, -1.0, 0, 0.0, 0.0, 0.0, 1.0)
glass = pb.loadURDF("glass2/urdf/glass.urdf", [-0.15, -0.6, 0.625],[0.0, 0.0, 0.0, 1.0])#, useFixedBase= 1)

kukaUid = objects[0]
''' Adjust to chnage robot position.....
kukaUid = objects[0]
pb.resetBasePositionAndOrientation(kukaUid, [0.00000, .2, 0.7],
                                      [0.000000, 0.000000, 0.000000, 1.000000])
'''
pb.resetBasePositionAndOrientation(kukaUid, [0.1, 0.1 , 0.2],
                                      [0.000000, 0.000000, 0.000000, 1.000000])
kukaEndEffectorIndex = 6
time.sleep(2)
pb.setRealTimeSimulation(1)

numJoints = pb.getNumJoints(kukaUid)


pos = [-0.25, -0.4, 0.9]
orn = [1.0, 0.0, 0.0, 1.0]

fingerAForce = 4
fingerBForce = 4
fingerTipForce = 4

temp = [0, 0, 0, 0, 0]
while(1):
  targetPos = []
  
  for i in range(len(paramIds)):
    c = paramIds[i]
    targetPos.append(pb.readUserDebugParameter(c))
  if(targetPos != temp):
    pos[0] = targetPos[0]
    pos[1] = targetPos[1]
    pos[2] = targetPos[2]
    jointPoses = pb.calculateInverseKinematics(kukaUid, 7, pos,orn)
    print("Joint Angles:", jointPoses)
    for i in range(7):
      pb.setJointMotorControl2(kukaUid, i, pb.POSITION_CONTROL, jointPoses[i])
    fingerAngle = targetPos[4]

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
    for i in range(len(targetPos)-1):
      temp[i]= targetPos[i]
    if(targetPos[5]>0.5):
      break

while(1):  
  targetPos = []
  for i in range(len(paramIds)):
      c = paramIds[i]
      targetPos.append(pb.readUserDebugParameter(c))
  if(targetPos != temp):
    pos[0] = targetPos[0]
    pos[1] = targetPos[1]
    pos[2] = targetPos[2]
    jointPoses = pb.calculateInverseKinematics(kukaUid, 7, pos)
    print("Joint Angles:", jointPoses)
    for i in range(7):
      pb.setJointMotorControl2(kukaUid, i, pb.POSITION_CONTROL, jointPoses[i])
  pb.setJointMotorControl2(kukaUid,
                          6,
                          pb.POSITION_CONTROL,
                          targetPosition=targetPos[3])


