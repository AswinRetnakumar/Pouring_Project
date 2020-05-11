import pybullet as pb
import pybullet_data
import time
import os
import cv2 
import numpy as np

path_data = '/home/aswin/Desktop/pouring_sim/pybullet_data1'
physicsClient = pb.connect(pb.GUI)
pb.setInternalSimFlags(0)

pb.setGravity(0,0,-9.8)

pb.setAdditionalSearchPath(pybullet_data.getDataPath())
plane = pb.loadURDF("plane.urdf")
table = pb.loadURDF(os.path.join(path_data, "table/table.urdf"), 0.0, 1.0, 0, 0.0, 0.0, 0.0, 1.0)

robotid = pb.loadURDF("new_mod/urdf/new_mod.urdf",[0.0,0.2,0.1] ,useFixedBase= 1)
glass = pb.loadURDF("glass2/urdf/glass.urdf", [-0.15, 0.6, 0.625],[0.0, 0.0, 0.0, 1.0], useFixedBase= 1)
#robotid = pb.loadURDF("kuka_iiwa/model.urdf")
numJoints = pb.getNumJoints(robotid)
jointIds = []
paramIds = []

img = True

viewMat = pb.computeViewMatrix(cameraEyePosition=[0.1, 0.5, 2.0],cameraTargetPosition= [0.0, 0.5, 0.75], cameraUpVector=[0,1.0,0])
projMatrix = pb.computeProjectionMatrixFOV(fov=45.0, aspect=1.0, nearVal=0.5, farVal=2.0)
pos = [0.0, 0.5, 0.75]
jointPoses = pb.calculateInverseKinematics(robotid, 6, pos)
for i in range(7):
    pb.setJointMotorControl2(robotid, i, pb.POSITION_CONTROL, jointPoses[i])
pb.setRealTimeSimulation(1)
time.sleep(0.1)
sphere_id = pb.loadURDF("pybullet_data1/sphere_1cm.urdf", [0.0, 0.65, 0.8])
pb.changeDynamics(sphere_id, -1, linearDamping=1.0, angularDamping=1.0, rollingFriction=0.0)


def main():

    pose = [0, 0, 0, 0, 0, 0, 0]

    
    paramIds.append(pb.addUserDebugParameter("x", -0.3, 0.3, 0.0 ))
    paramIds.append(pb.addUserDebugParameter("y", 0.5, 0.7, 0.5 ))
    paramIds.append(pb.addUserDebugParameter("z", 0.75, 0.9, 0.75 ))
    paramIds.append(pb.addUserDebugParameter("glass_ang", -2.9, 2.9, 0.0 ))
    
    temp = [0, 0, 0, 0, 0]
    pos = [0,0,0]

    while(1):  

        if img:
            _,_,img_arr,_,seg = pb.getCameraImage(width=256,
                               height=256,
                               viewMatrix=viewMat,
                               projectionMatrix=projMatrix,
                               renderer = pb.ER_TINY_RENDERER)
        '''closestPoints = pb.getClosestPoints(glass, sphere_id, 0.01, -1)
        print("Points ", closestPoints)'''
        #cv2.imwrite("test.jpg", img_arr[:,:,:3])
        pos_ee = pb.getLinkState(robotid, 6)
        print(pos_ee[0])
        gnd_collision = True if len(pb.getClosestPoints(sphere_id, table, 0.01))> 0 else False
        glass_collision = True if len(pb.getClosestPoints(sphere_id, glass, 0.01))+ len(pb.getClosestPoints(robotid , glass, 0.01))>0 else False
        spherePos, _ = pb.getBasePositionAndOrientation(sphere_id)
        s = 0
        for i, j in zip(spherePos, (-0.15, 0.6,0.641)):
            s += (j-i)**2
        s = s**0.5
        capture = True if s< 0.015 else False
        print("capture "+str(capture))

        print("gnd"+str(gnd_collision))
        print("glas"+str(glass_collision))
        targetPos = []
        for i in range(len(paramIds)):
            c = paramIds[i]
            targetPos.append(pb.readUserDebugParameter(c))
        if(targetPos != temp):
            pos[0] = targetPos[0]
            pos[1] = targetPos[1]
            pos[2] = targetPos[2]
            jointPoses = pb.calculateInverseKinematics(robotid, 6, pos)
            for i in range(7):
                pb.setJointMotorControl2(robotid, i, pb.POSITION_CONTROL, jointPoses[i])
        pb.setJointMotorControl2(robotid,
                                6,
                                pb.POSITION_CONTROL,
                                targetPosition=targetPos[3])




if __name__ == "__main__":
    main()
