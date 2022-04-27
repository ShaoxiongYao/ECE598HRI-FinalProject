import klampt
import time

from klampt import WorldModel
from klampt import vis

from klampt.model import ik
from klampt.plan import robotplanning

def get_home_config():
    q_home = [0]*29
    q_home[7] += 1
    return q_home

if __name__ == '__main__':
    # load robot arm
    world = WorldModel()
    world.readFile("TRINA_Model/urdf/TRINA_Defender_v2.urdf")

    # save robot reference
    robot = world.robot(0)

    vis.add("world", world)
    vis.show()

    q_home = get_home_config()
    robot.setConfig(q_home)

    # qgoal = robot.getConfig()
    # q_goal[8] -= 1.0   #move the 4th joint 2 radians from the start
    # # ignore_lst = [('torso_link', 'left_base_link'), ('torso_link' right_base_link collide)
    # planner = robotplanning.plan_to_config(world, robot, q_goal, ignoreCollisions=[])
    # path = planner.getPath()
    # print("path type:", type(path))

    # solve_nearby
    # activeDofs = arm_dof

    # camera_frame to robot_frame

    # # space = RobotCSpace(robot,collide.WorldCollider(world))
    # # # (Can also create it without the collider to ignore all self-and environment collisions)
    # # #Optionally:
    # # #Call space.addFeasibilityTest(func,name=None) on the space with as many additional feasibility tests as you want
    # # qinit = robot.getConfig()
    # # qgoal = robot.getConfig()
    # # qgoal[7] += 2.0       #move 2 radians on joint 3

    # print("q goal len:", len(qgoal))
    
    # left_arm_joints = list(range(7, 14))
    left_arm_joints = list(range(15, 29))
    for joint_idx in left_arm_joints:
        print("move joint:", joint_idx)
        for _ in range(100):
            q_home[joint_idx] += 0.1
            robot.setConfig(q_home)
            time.sleep(0.1)

    left_wrist = robot.link('left_wrist1_link')
    world1 = left_wrist.getWorldPosition([0, 0, 0])
    world2 = left_wrist.getWorldPosition([0, 0, 1])
    world1[0] += 0.01
    world2[0] += 0.01
    print("world1:", world1)
    print("world2:", world2)

    left_obj = ik.objective(left_wrist, local=[[0, 0, 0],[0, 0, 1]], 
                            world=[world1,world2])

    print("before config:", robot.getConfig())
    ik.solve_nearby(left_obj, 0.1,
                    iters=1000, tol=1e-3, activeDofs=left_arm_joints,
                    numRestarts = 0, feasibilityCheck=None)
    print("after config:", robot.getConfig())

    time.sleep(10)
