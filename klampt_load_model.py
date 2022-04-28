import time

import pickle
import klampt
import numpy as np
from klampt import WorldModel, vis
from klampt.model import ik, geometry
from klampt.plan import robotplanning
from klampt.io import load
from klampt.io import open3d_convert, numpy_convert
import open3d as o3d


def init_robot(robot):

    q_home = [0]*29
    q_home[7] += 1.0
    q_home[8] -= 1.5
    # q_home[15] += 1.0
    q_home[16] -= 2.0
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

def load_hand_seq(gt_data_path, key):

    with open(gt_data_path, 'rb') as f:
        data_df = pickle.load(f)
    return data_df[key]


R_I3 = [1, 0, 0, 0, 1, 0, 0, 0, 1]

if __name__ == '__main__':
    # load robot arm
    world = WorldModel()
    world.readFile("TRINA_Model/urdf/TRINA_Defender_v2.urdf")

    # save robot reference
    robot = world.robot(0)

    vis.add("world", world)
    vis.show()

    robot = init_robot(robot)

    R = [0, 1,  0,  0,  0, -1,  -1,  0,  0]
    t = [0, 0, 1.2]
    pc = load('auto', 'Dataset/cam_torso.pcd')
    pc.setCurrentTransform(R, t)
    hand_pc = load('auto', 'Dataset/hand_pcd.pcd')
    hand_pc.setCurrentTransform(R, t)

    # vertices = hand_pc.getPointCloud().vertices
    # for i in range(100):
    #     print(f"vertices {i}:", vertices[i])
    # input()

    vis.add('cam', pc)
    vis.add('hand_pcd', hand_pc)

    q_home = robot.getConfig()
    right_arm_joints = list(range(16, 22))
    # for joint_idx in right_arm_joints:
    #     print("move joint:", joint_idx)
    #     for _ in range(100):
    #         q_home[joint_idx] += 0.1
    #         robot.setConfig(q_home)
    #         time.sleep(0.1)

    right_wrist = robot.link('right_EE_link')

    hand_center = np.array([0.5, 0.5, 0.5])
    robot_com = np.array(robot.getCom())
    
    world_normal = hand_center - robot_com
    world_normal /= np.linalg.norm(world_normal)

    local_normal = np.array(right_wrist.getLocalDirection(world_normal))

    print("local_normal:", local_normal)
    local_p0 = np.array([0, 0, 0])
    world_p0 = np.array(right_wrist.getWorldPosition([0, 0, 0]))

    local_p1_box = geometry.box(0.05, 0.05, 0.05, center=world_p0)
    vis.add('local_p1_box', local_p1_box)
    world_p1_box = geometry.box(0.05, 0.05, 0.05, center=world_p0)
    vis.add('world_p1_box', world_p1_box)
    local_p2_box = geometry.box(0.05, 0.05, 0.05, center=world_p0)
    vis.add('local_p2_box', local_p2_box)
    world_p2_box = geometry.box(0.05, 0.05, 0.05, center=world_p0)
    vis.add('world_p2_box', world_p2_box)

    key = 'gt_left_hand'
    hand_seq = load_hand_seq('Dataset/first_ground_truth_alice_no_sword1.pkl', key)

    for step_idx, hand_center in enumerate(hand_seq[:10]):
        hand_box = geometry.box(0.05, 0.05, 0.05, center=hand_center)
        hand_box.transform(R, t)
        vis.add(f'hand_box{step_idx}', hand_box)

    local_p1 = local_p0.copy()
    world_p1 = world_p0.copy()

    for step_idx in range(10):

        local_p1_inW = np.array(right_wrist.getWorldPosition(local_p1))
        # local_p1_box.transform(R_I3, local_p1_inW)

        world_p1 += np.array([-0.01*step_idx, 0.001*step_idx, 0.0])
        print("world p1:", world_p1)
        world_p1_box.setCurrentTransform(R_I3, [-0.01, 0.001, 0.0])
        vis.add(f'world_p1_step{step_idx}', geometry.box(0.05, 0.05, 0.05, center=world_p1))

        lambda1 = 0.1
        local_p2 = local_p1 + lambda1*local_normal
        world_p2 = world_p1 + lambda1*world_normal

        local_p2_inW = np.array(right_wrist.getWorldPosition(local_p2))
        # local_p2_box.transform(R_I3, local_p2_inW)
        # world_p2_box.transform(R_I3, world_p2)

        ik_obj = ik.objective(right_wrist, local=[local_p1,local_p2], 
                              world=[world_p1,world_p2])

        print("before config:", robot.getConfig())
        # solve_result = ik.solve_nearby(ik_obj, 0.1, iters=1000, tol=1e-3,
        #                                activeDofs=right_arm_joints,
        #                                numRestarts=0, feasibilityCheck=None)
        solve_result = ik.solve_global(ik_obj, iters=1000, tol=1e-3, activeDofs=right_arm_joints,
                                       numRestarts = 100, feasibilityCheck = None, startRandom = False )
        print("ik result:", solve_result)
        print("after config:", robot.getConfig())

        vis.update()
        time.sleep(2)
    time.sleep(100)

        # q_goal = robot.getConfig()
        # q_goal[17] += 1.0
        # planner = robotplanning.planToConfig(world, robot, q_goal)
        # print("planner:", planner)
        # q_path = planner.getPath()
        # print("q path:", q_path)
