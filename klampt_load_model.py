import time

import pickle
import klampt
import numpy as np
import pandas as pd
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
    vis.add('cam', pc)

    right_arm_joints = list(range(16, 21))
    right_EE_link = robot.link('right_shield_link')

    robot_com = np.array(robot.getCom())
    right_shoulder_link = robot.link('right_shoulder_link')
    reach_center = np.array(right_shoulder_link.getWorldPosition([0, 0, 0]))
    print('reach center:', reach_center)
    
    local_p1_box = geometry.box(0.05, 0.05, 0.05, center=[0, 0, 0])
    vis.add('local_p1_box', local_p1_box)
    world_p1_box = geometry.box(0.05, 0.05, 0.05, center=[0, 0, 0])
    vis.add('world_p1_box', world_p1_box)
    local_p2_box = geometry.box(0.05, 0.05, 0.05, center=[0, 0, 0])
    vis.add('local_p2_box', local_p2_box, color=[1, 0, 0])
    world_p2_box = geometry.box(0.05, 0.05, 0.05, center=[0, 0, 0])
    vis.add('world_p2_box', world_p2_box, color=[1, 0, 0])
    hand_box = geometry.box(0.05, 0.05, 0.05, center=[0, 0, 0])
    vis.add(f'hand_box', hand_box)

    df = pd.read_pickle('Dataset/first_ground_truth_alice_no_sword1.pkl')

    color_path_seq = df['cam_torso_color']
    depth_path_seq = df['cam_torso_depth']
    hand_seq = df['gt_left_hand']

    local_p1 = [0, 0, 0]

    # distance to clamp point position
    clamp_thres = 0.7

    feasible_check = lambda : not robot.selfCollides()
    prev_hand_center = None

    for step_idx, hand_center in enumerate(hand_seq):
        print("step:", step_idx)
        trans_center = np.array(R).reshape(3, 3).T @ hand_center + np.array(t) 
        hand_box.setCurrentTransform(R_I3, trans_center)

        if prev_hand_center is not None:
            hand_diff_l2 = np.linalg.norm(trans_center - prev_hand_center)

            # ignore spurious detection
            if hand_diff_l2 > 0.5:
                continue
        prev_hand_center = trans_center

        local_p1_inW = np.array(right_EE_link.getWorldPosition(local_p1))
        local_p1_box.setCurrentTransform(R_I3, local_p1_inW)

        world_normal = trans_center - robot_com
        world_normal /= np.linalg.norm(world_normal)

        world_p1 = trans_center
        if np.linalg.norm(world_p1-reach_center) > clamp_thres:
            reach_normal = world_p1 - reach_center
            reach_normal /= np.linalg.norm(reach_normal)
            world_p1 = reach_center + clamp_thres*reach_normal
        world_p1_box.setCurrentTransform(R_I3, world_p1)

        lambda1 = 0.1
        local_p2 = local_p1 + lambda1*np.array([0, 0, 1])
        local_p2_inW = np.array(right_EE_link.getWorldPosition(local_p2))
        local_p2_box.setCurrentTransform(R_I3, local_p2_inW)
        world_p2 = world_p1 + lambda1*world_normal
        world_p2_box.setCurrentTransform(R_I3, world_p2)

        local_p2_inW = np.array(right_EE_link.getWorldPosition(local_p2))

        ik_obj = ik.objective(right_EE_link, local=[local_p1,local_p2], 
                              world=[world_p1,world_p2])

        prev_config = robot.getConfig()
        vis.lock()
        solve_result = ik.solve_global(ik_obj, iters=1000, tol=1e-3, activeDofs=right_arm_joints,
                                       numRestarts=100, feasibilityCheck=feasible_check, startRandom=False)
        curr_config = robot.getConfig()
        if not solve_result:
            robot.setConfig(prev_config)
        vis.unlock()
        print("ik result:", solve_result)

        vis.update()
        time.sleep(0.01)
