import klampt
import time

from klampt import WorldModel
from klampt import vis

from klampt.model import ik
from klampt.plan import robotplanning

def init_robot(robot):

    q_home = [0]*29
    q_home[7] += 1.0
    q_home[8] -= 1.5
    q_home[15] += 1.0
    q_home[16] -= 1.5
    robot.setConfig(q_home)

    # torso_link: 25
    # left_base_link: 6
    # right_base_link: 15
    robot.enableSelfCollision(6, 25, False)
    robot.enableSelfCollision(15, 25, False)

    # right_EE_link: 21
    # right_shielf_link: 23
    robot.enableSelfCollision(21, 23, False)
    return robot


if __name__ == '__main__':
    # load robot arm
    world = WorldModel()
    world.readFile("TRINA_Model/urdf/TRINA_Defender_v2.urdf")

    # save robot reference
    robot = world.robot(0)

    vis.add("world", world)
    vis.show()

    robot = init_robot(robot)
    # time.sleep(10)

    q_home = robot.getConfig()
    
    # left_arm_joints = list(range(7, 14))
    right_arm_joints = list(range(16, 22))
    for joint_idx in right_arm_joints:
        print("move joint:", joint_idx)
        for _ in range(100):
            q_home[joint_idx] += 0.1
            robot.setConfig(q_home)
            time.sleep(0.1)

    right_wrist = robot.link('right_EE_link')
    world1 = right_wrist.getWorldPosition([0, 0, 0])
    world2 = right_wrist.getWorldPosition([0, 0, 1])
    world1[0] += 0.01
    world2[0] += 0.01
    print("world1:", world1)
    print("world2:", world2)

    left_obj = ik.objective(right_wrist, local=[[0, 0, 0],[0, 0, 1]], 
                            world=[world1,world2])

    print("before config:", robot.getConfig())
    ik.solve_nearby(left_obj, 0.1,
                    iters=1000, tol=1e-3, activeDofs=right_arm_joints,
                    numRestarts = 0, feasibilityCheck=None)
    print("after config:", robot.getConfig())

    time.sleep(10)
