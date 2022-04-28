import numpy as np
import open3d as o3d

from klampt_load_model import load_hand_seq

R = [0, 1,  0,  0,  0, -1,  -1,  0,  0]
t = [0, 0, 1.2]
H = np.array([[0, 1, 0, 0],
              [0, 0, -1, 0],
              [-1, 0, 0, 1.2],
              [0, 0, 0, 1]])

if __name__ == '__main__':
    E_dir = 'Calibration/data/extrinsics'

    cam_left_pcd = o3d.io.read_point_cloud('Dataset/cam_left.pcd')
    E_left2torso = np.load(E_dir+'/left2torso.npy')
    cam_left_pcd.transform(E_left2torso)

    cam_right_pcd = o3d.io.read_point_cloud('Dataset/cam_right.pcd')
    E_right2torso = np.load(E_dir+'/right2torso.npy')
    cam_right_pcd.transform(E_right2torso)

    cam_torso_pcd = o3d.io.read_point_cloud('Dataset/cam_torso.pcd')
    cam_torso_pcd.transform(H)

    create_hand_pcd = True
    if create_hand_pcd:
        key = 'gt_left_hand'
        hand_seq = load_hand_seq('Dataset/first_ground_truth_alice_no_sword1.pkl', key)
        hand_seq = [np.array(R).reshape(3, 3) @ pt + np.array(t) 
                    for pt in hand_seq]
        print("hand seq:", hand_seq[:10])

        hand_pcd = o3d.geometry.PointCloud()
        hand_pcd.points = o3d.utility.Vector3dVector(hand_seq)
        hand_pcd.paint_uniform_color([1., 0., 0.])

        # o3d.io.write_point_cloud('Dataset/hand_pcd.pcd', hand_pcd)
    else:
        hand_pcd = o3d.io.read_point_cloud('Dataset/hand_pcd.pcd')

    o3d.visualization.draw_geometries([cam_torso_pcd, hand_pcd])
