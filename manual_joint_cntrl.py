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
pb.resetBasePositionAndOrientation(kukaUid, [0.0, 0.1 , 0.2],
                                      [0.000000, 0.000000, 0.000000, 1.000000])
kukaEndEffectorIndex = 6
numJoints = pb.getNumJoints(kukaUid)
pb.setRealTimeSimulation(1)

numJoints = pb.getNumJoints(kukaUid)


pos = [-0.25, -0.4, 0.9]
orn = [1.0, 0.0, 0.0, 1.0]


jointPositions = [
        1.6, 0.0 , 0.0, 1.642, 1.34, 1.0, -0.00, 0.0,
        0.0]
for i in range(9):
  pb.setJointMotorControl2(kukaUid, i, pb.POSITION_CONTROL, jointPositions[i])
time.sleep(2)
jointPoses = pb.calculateInverseKinematics(kukaUid, 7, pos,
                                                    orn)#, ll, ul, jr, rp)
print("Num joints:", numJoints)

pose = [0, 0, 0, 0, 0, 0, 0, 0.0, 0.0, 0.0, 0.0]


for j in range(7):

  pose[j] = jointPoses[j]

print("Angles: ", pose)
for i in range(11):
  pb.setJointMotorControl2(kukaUid, i, pb.POSITION_CONTROL, pose[i])#, force=5 * 100.)
time.sleep(1)

for j in range(numJoints):
  pb.changeDynamics(kukaUid, j, linearDamping=0, angularDamping=0)
  info = pb.getJointInfo(kukaUid, j)
  jointName = info[1]
  jointType = info[2]
  start_pos = pose + [0.0, 0.0, 0.0]
  if (jointType == pb.JOINT_PRISMATIC or jointType == pb.JOINT_REVOLUTE):
    jointIds.append(j)
    paramIds.append(pb.addUserDebugParameter(jointName.decode("utf-8"), -4, 4, start_pos[j] ))

  
 
while (1):
  for i in range(len(paramIds)):
    c = paramIds[i]
    targetPos = pb.readUserDebugParameter(c)
    pb.setJointMotorControl2(kukaUid, jointIds[i], pb.POSITION_CONTROL, targetPos, force=5 * 1000.)
  time.sleep(0.01)
