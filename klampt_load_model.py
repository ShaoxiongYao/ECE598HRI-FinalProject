import klampt
import time

from klampt import WorldModel
from klampt import vis

from klampt.plan import robotplanning

if __name__ == '__main__':
    # load robot arm
    world = WorldModel()
    world.readFile("TRINA_Model/urdf/TRINA_Defender.urdf")

    # save robot reference
    robot = world.robot(0)

    vis.add("world", world)
    vis.show()

    qgoal = robot.getConfig()
    # qgoal[3] += 2.0   #move the 4th joint 2 radians from the start
    # planner = robotplanning.planToConfig(world,robot,qgoal)
    # path = planner.getPath()
    # print("path type:", type(path))
    
    print("q goal len:", len(qgoal))
    
    left_arm_joints = list(range(7, 14))
    for joint_idx in left_arm_joints:
        print("move joint:", joint_idx)
        for _ in range(100):
            qgoal[joint_idx] += 0.1
            robot.setConfig(qgoal)
            time.sleep(0.1)
