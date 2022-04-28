import time

import pickle
import klampt
import numpy as np
import pandas as pd
from klampt import WorldModel, vis
from klampt.model import ik, geometry
from klampt.plan import robotplanning
import cv2
from klampt.io import load
from klampt.io import open3d_convert, numpy_convert
import open3d as o3d
from create_point_cloud import load_point_cloud


def init_robot(robot):

    qmin, qmax = robot.getJointLimits()
    qmin[26] = 0
    qmax[26] = 2*np.pi
    robot.setJointLimits(qmin, qmax)

    q_home = [0]*29
    q_home[7] += 1.0
    q_home[8] -= 1.5
    q_home[16] -= 1.0
    # q_home[18] += 1.0
    q_home[26] += np.pi
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

    R = [0, -1,  0,  0,  0, -1,  1,  0,  0]
    t = [0, 0, 1.2]
    pc = load('auto', 'Dataset/cam_torso.pcd')
    pc.setCurrentTransform(R, t)
    vis.add('cam', pc)

    right_arm_joints = list(range(16, 21))
    right_EE_link = robot.link('right_shield_link')

    robot_com = np.array(robot.getCom())
    right_shoulder_link = robot.link('right_shoulder_link')
    reach_center = np.array(right_shoulder_link.getWorldPosition([0, 0, 0]))

    time.sleep(10)

    debug_pts = False
    if debug_pts:
        local_p1_box = geometry.box(0.05, 0.05, 0.05, center=[0, 0, 0])
        vis.add('local_p1_box', local_p1_box)
        world_p1_box = geometry.box(0.05, 0.05, 0.05, center=[0, 0, 0])
        vis.add('world_p1_box', world_p1_box)
        local_p2_box = geometry.box(0.05, 0.05, 0.05, center=[0, 0, 0])
        vis.add('local_p2_box', local_p2_box, color=[1, 0, 0])
        world_p2_box = geometry.box(0.05, 0.05, 0.05, center=[0, 0, 0])
        vis.add('world_p2_box', world_p2_box, color=[1, 0, 0])
    hand_box = geometry.box(0.05, 0.05, 0.05, center=[0, 0, 0])
    vis.add(f'hand_box', hand_box, color=[1, 0, 0])

    df = pd.read_pickle('Dataset/first_ground_truth_alice_no_sword1.pkl')

    color_path_seq = df['cam_right_color']
    depth_path_seq = df['cam_right_depth']
    hand_seq = df['gt_right_hand']

    local_p1 = [0, 0, 0]

    # distance to clamp point position
    clamp_thres = 0.7

    feasible_check = lambda : not robot.selfCollides()
    prev_hand_center = None

    dataset_dir = "/media/yaosx/8AF1-B496/HRI_dataset"

    fx, fy, cx, cy = 931.43615398, 939.4127871, 619.07725813, 319.53233415

    E_right2torso = np.load('Calibration/data/extrinsics/right2torso.npy')

    for step_idx, hand_center in enumerate(hand_seq[:200]):
        print("step:", step_idx)

        color_path = color_path_seq[step_idx].format(dataset_dir)
        color_path = color_path.replace("torso", "right")
        depth_path = depth_path_seq[step_idx].format(dataset_dir)
        depth_path = depth_path.replace("torso", "right")

        o3d_pcd = load_point_cloud(color_path, depth_path, 'realsense_right')
        o3d_pcd.transform(E_right2torso)
        klampt_pc = open3d_convert.from_open3d(o3d_pcd)
        klampt_pc.transform(R, t)
        vis.add('cam', klampt_pc)

        trans_center = np.array(R).reshape(3, 3).T @ hand_center + np.array(t) 
        hand_box.setCurrentTransform(R_I3, trans_center)

        if prev_hand_center is not None:
            hand_diff_l2 = np.linalg.norm(trans_center - prev_hand_center)

            # ignore spurious detection
            if hand_diff_l2 > 2.0:
                continue
        prev_hand_center = trans_center

        local_p1_inW = np.array(right_EE_link.getWorldPosition(local_p1))
        if debug_pts:
            local_p1_box.setCurrentTransform(R_I3, local_p1_inW)

        world_normal = trans_center - robot_com
        world_normal /= np.linalg.norm(world_normal)

        world_p1 = trans_center
        if np.linalg.norm(world_p1-reach_center) > clamp_thres:
            reach_normal = world_p1 - reach_center
            reach_normal /= np.linalg.norm(reach_normal)
            world_p1 = reach_center + clamp_thres*reach_normal
        if debug_pts:
            world_p1_box.setCurrentTransform(R_I3, world_p1)

        lambda1 = 0.1
        local_p2 = local_p1 + lambda1*np.array([0, 0, 1])
        world_p2 = world_p1 + lambda1*world_normal
        if debug_pts:
            local_p2_inW = np.array(right_EE_link.getWorldPosition(local_p2))
            local_p2_box.setCurrentTransform(R_I3, local_p2_inW)
            world_p2_box.setCurrentTransform(R_I3, world_p2)

        local_p2_inW = np.array(right_EE_link.getWorldPosition(local_p2))

        ik_obj = ik.objective(right_EE_link, local=[local_p1,local_p2], 
                              world=[world_p1,world_p2])

        prev_config = robot.getConfig()
        vis.lock()
        # solve_result = ik.solve_global(ik_obj, iters=1000, tol=1e-3, activeDofs=right_arm_joints,
        #                                numRestarts=100, feasibilityCheck=feasible_check, startRandom=False)
        solve_result = ik.solve_nearby(ik_obj, 2.0, iters=1000, tol=1e-3, 
                                       activeDofs=right_arm_joints,
                                       numRestarts=100, feasibilityCheck=feasible_check)
        
        curr_config = robot.getConfig()
        if not solve_result:
            robot.setConfig(prev_config)
        vis.unlock()
        print("ik result:", solve_result)

        vis.update()
        time.sleep(0.01)
