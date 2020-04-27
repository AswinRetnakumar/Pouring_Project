import pybullet as pb
import pybullet_data
import time
import os

path_data = '/home/aswin/Desktop/pouring_sim/pybullet_data1'
physicsClient = pb.connect(pb.GUI)
pb.setInternalSimFlags(0)

pb.setGravity(0,0,-9.8)
pb.setAdditionalSearchPath(pybullet_data.getDataPath())
plane = pb.loadURDF("plane.urdf")
pb.loadURDF(os.path.join(path_data, "table/table.urdf"), 0.0, 1.0, 0, 0.0, 0.0, 0.0, 1.0)

robotid = pb.loadURDF("new_mod/urdf/new_mod.urdf",[0.0,0.2,0.1] ,useFixedBase= 1)

#robotid = pb.loadURDF("kuka_iiwa/model.urdf")
numJoints = pb.getNumJoints(robotid)
print("Number of Joints: ", numJoints)
jointIds = []
paramIds = []

viewMat = pb.computeViewMatrix([0.0, 0.5, 2.5], [0.0, 0.5, 0.75], [0,1,0])
projMatrix = pb.computeProjectionMatrixFOV(fov=45.0, aspect=1.0, nearVal=0.5, farVal=2.6)
pos = [0.0, 0.5, 0.75]
jointPoses = pb.calculateInverseKinematics(robotid, 6, pos)
for i in range(7):
    pb.setJointMotorControl2(robotid, i, pb.POSITION_CONTROL, jointPoses[i])
pb.setRealTimeSimulation(1)
time.sleep(0.1)
sphere_id = pb.loadURDF("pybullet_data1/sphere_1cm.urdf", [0.0, 0.65, 0.8])


def main():

    pose = [0, 0, 0, 0, 0, 0, 0]

    
    paramIds.append(pb.addUserDebugParameter("x", -0.3, 0.3, 0.0 ))
    paramIds.append(pb.addUserDebugParameter("y", 0.5, 0.7, 0.5 ))
    paramIds.append(pb.addUserDebugParameter("z", 0.75, 0.9, 0.75 ))
    paramIds.append(pb.addUserDebugParameter("glass_ang", -2.9, 2.9, 0.0 ))
    
    temp = [0, 0, 0, 0, 0]
    pos = [0,0,0]

    while(1):  
        img_arr = pb.getCameraImage(width=256,
                               height=256,
                               viewMatrix=viewMat,
                               projectionMatrix=projMatrix)
        targetPos = []
        for i in range(len(paramIds)):
            c = paramIds[i]
            targetPos.append(pb.readUserDebugParameter(c))
        if(targetPos != temp):
            pos[0] = targetPos[0]
            pos[1] = targetPos[1]
            pos[2] = targetPos[2]
            jointPoses = pb.calculateInverseKinematics(robotid, 6, pos)
            print("Joint Angles:", jointPoses)
            for i in range(7):
                pb.setJointMotorControl2(robotid, i, pb.POSITION_CONTROL, jointPoses[i])
        pb.setJointMotorControl2(robotid,
                                6,
                                pb.POSITION_CONTROL,
                                targetPosition=targetPos[3])




if __name__ == "__main__":
    main()
