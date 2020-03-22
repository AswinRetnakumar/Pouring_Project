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
ll = [-.967, -2, -2.96, 0.19, -2.96, -2.09, -3.05]
    #upper limits for null space
ul = [.967, 2, 2.96, 2.29, 2.96, 2.09, 3.05]
    #joint ranges for null space
jr = [5.8, 4, 5.8, 4, 5.8, 4, 6]
    #restposes for null space
rp = [0, 0, 0, 0.5 * math.pi, 0, -math.pi * 0.5 * 0.66, 0]
    #joint damping coefficents
jd = [ 0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001,
        0.00001, 0.00001, 0.00001, 0.00001]
'''
jointPositions = [
        2.96, 0.1 , -0.011401, -1.589317, 0.005379, 1.137684, -0.006539, 0.1,
        -0.299912, 0.000000, -0.000043, 0.299960, 0.000000, -0.000200
    ]

numJoints = pb.getNumJoints(kukaUid)
print(numJoints)
maxForce = 200.
for jointIndex in range(numJoints):
      pb.resetJointState(kukaUid, jointIndex, jointPositions[jointIndex])
      pb.setJointMotorControl2(kukaUid,
                              jointIndex,
                              pb.POSITION_CONTROL,
                              targetPosition=jointPositions[jointIndex],
                              force=maxForce)
      

'''

time.sleep(2)
'''
pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 0)
sphere_id = []
for k in range(2):
  for l in range(3):
    for m in range(10):
      sphere_id.append(pb.loadURDF("pybullet_data1/sphere_1cm.urdf", [-0.148+0.01*k, -0.59-0.01*l, 0.64+ 0.01*m], [0.0, 0.0, 0.0, 1.0]))
pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 1)
'''

'''
for id in sphere_id:
  pb.changeDynamics(id, -1, linearDamping=1.0, angularDamping=1.0)
'''


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

fingerAForce = 4
fingerBForce = 4
fingerTipForce = 4

for j in range(7):

  pose[j] = jointPoses[j]

print("Angles: ", pose)
for i in range(11):
  pb.setJointMotorControl2(kukaUid, i, pb.POSITION_CONTROL, pose[i])#, force=5 * 100.)
time.sleep(1)

paramIds.append(pb.addUserDebugParameter("x", -0.3, 0.3, -0.15 ))
paramIds.append(pb.addUserDebugParameter("y", -1.4, 1.0, -0.32 ))
paramIds.append(pb.addUserDebugParameter("z", 0.64, 1.1, 0.66 ))
paramIds.append(pb.addUserDebugParameter("orientation", -2.0, 2.0, 0.0 ))
paramIds.append(pb.addUserDebugParameter("gripper",0.06,0.5,0.4))
paramIds.append(pb.addUserDebugParameter("control",0.0,1.0,0.0))

'''
while(1):
   
  targetPos = []
  for i in range(len(paramIds)):
    c = paramIds[i]
    targetPos.append(pb.readUserDebugParameter(c))
  fingerAngle = targetPos[1]
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
                                targetPosition=0,
                                force=fingerTipForce)
  pb.setJointMotorControl2(kukaUid,
                                13,
                                pb.POSITION_CONTROL,
                                targetPosition=0,
                                force=fingerTipForce)
  
'''

"""
for j in range(numJoints):
  pb.changeDynamics(kukaUid, j, linearDamping=0, angularDamping=0)
  info = pb.getJointInfo(kukaUid, j)
  jointName = info[1]
  jointType = info[2]
  start_pos = pose + [0.0, 0.0, 0.0]
  if (jointType == pb.JOINT_PRISMATIC or jointType == pb.JOINT_REVOLUTE):
    jointIds.append(j)
    paramIds.append(pb.addUserDebugParameter(jointName.decode("utf-8"), -4, 4, start_pos[j] ))
"""
  
  
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
print("Control orientation...")
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




'''
while (1):
  for i in range(len(paramIds)):
    c = paramIds[i]
    targetPos = pb.readUserDebugParameter(c)
    pb.setJointMotorControl2(kukaUid, jointIds[i], pb.POSITION_CONTROL, targetPos, force=5 * 1000.)
  time.sleep(0.01)
'''