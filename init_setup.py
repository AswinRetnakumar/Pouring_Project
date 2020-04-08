import pybullet as pb
import pybullet_data
import os
import time
import sys

kukaEndEffectorIndex = 6

fingerAForce = 4
fingerBForce = 4
fingerTipForce = 4
physicsClient = pb.connect(pb.GUI)


def load_data():
    
    pb.setGravity(0,0,-9.8)
    pb.setAdditionalSearchPath(pybullet_data.getDataPath())
    plane = pb.loadURDF("plane.urdf")

    path_data = '/home/aswin/Desktop/pouring_sim/pybullet_data1'
    pb.setGravity(0,0,-9.8)


    pb.setInternalSimFlags(0)

    objects = pb.loadSDF(os.path.join(path_data, "kuka_iiwa/kuka_with_gripper.sdf"))
    pb.loadURDF(os.path.join(path_data, "table/table.urdf"), 0.0, -1.0, 0, 0.0, 0.0, 0.0, 1.0)
    glass = pb.loadURDF("glass2/urdf/glass.urdf", [-0.15, -0.6, 0.625],[0.0, 0.0, 0.0, 1.0])#, useFixedBase= 1)

    kukaUid = objects[0]

    pb.resetBasePositionAndOrientation(kukaUid, [0.1, 0.1 , 0.2],
                                        [0.000000, 0.000000, 0.000000, 1.000000])
    pb.setRealTimeSimulation(1)

    numJoints = pb.getNumJoints(kukaUid)
    
    return kukaUid




def load_spheres():

    pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 0)
    
    sphere_id = []
    for k in range(2):
        for l in range(3):
            for m in range(10):
                sphere_id.append(pb.loadURDF("pybullet_data1/sphere_1cm.urdf", [-0.148+0.01*k, -0.59-0.01*l, 0.64+ 0.01*m], [0.0, 0.0, 0.0, 1.0]))
    
    pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 1)


def main():
    load_data()
    time.sleep(3)
    if int(sys.argv[1]) == 1:
        load_spheres()
        time.sleep(5)

if __name__ == "__main__":
    main()