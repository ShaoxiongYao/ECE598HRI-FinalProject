import glob
import os
import pickle

import numpy as np
import open3d as o3d

# w, h: 1920, 1090

def load_cam_K(K_fn, W, H):
    with open(K_fn, 'rb') as f:
        K_dict = pickle.load(f)
    
    cam_K_dict = {}
    for k, v in K_dict.items():
        K_mat = v[0]

        fx, fy = K_mat[0, 0], K_mat[1, 1]
        cx, cy = K_mat[0, 2], K_mat[1, 2]
        cam_K = o3d.camera.PinholeCameraIntrinsic(W, H, fx, fy, cx, cy)
        cam_K_dict[k] = cam_K
    return cam_K_dict

def get_image_names(folder):
    cam_name_lst = ['cam_left', 'cam_right', 'cam_torso']
    img_fn_dict = {}
    for cam_name in cam_name_lst:
        img_fn_dict[cam_name] = {}
        color_folder = os.path.join(folder, cam_name, 'color')
        img_fn_dict[cam_name]['color_fn_lst'] = sorted(glob.glob(color_folder+'/*'))
        depth_folder = os.path.join(folder, cam_name, 'depth')
        img_fn_dict[cam_name]['depth_fn_lst'] = sorted(glob.glob(depth_folder+'/*'))
        
    return img_fn_dict
