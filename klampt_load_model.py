import klampt
import time

from klampt import WorldModel
from klampt import vis

from klampt.plan import robotplanning

def get_home_config():
    q_home = [0]*29
    q_home[7] += 1
    return q_home

if __name__ == '__main__':
    # load robot arm
    world = WorldModel()
    world.readFile("TRINA_Model/urdf/TRINA_Defender.urdf")

    # save robot reference
    robot = world.robot(0)

    vis.add("world", world)
    vis.show()

    q_home = get_home_config()
    robot.setConfig(q_home)
    time.sleep(10)

    q_goal = robot.getConfig()
    q_goal[8] -= 1.0   #move the 4th joint 2 radians from the start
    planner = robotplanning.planToConfig(world, robot, q_goal)
    path = planner.getPath()
    print("path type:", type(path))
    input()

    # space = RobotCSpace(robot,collide.WorldCollider(world))
    # # (Can also create it without the collider to ignore all self-and environment collisions)
    # #Optionally:
    # #Call space.addFeasibilityTest(func,name=None) on the space with as many additional feasibility tests as you want
    # qinit = robot.getConfig()
    # qgoal = robot.getConfig()
    # qgoal[7] += 2.0       #move 2 radians on joint 3

    print("q goal len:", len(qgoal))
    
    left_arm_joints = list(range(7, 14))
    for joint_idx in left_arm_joints:
        print("move joint:", joint_idx)
        for _ in range(100):
            qgoal[joint_idx] += 0.1
            robot.setConfig(qgoal)
            time.sleep(0.1)
