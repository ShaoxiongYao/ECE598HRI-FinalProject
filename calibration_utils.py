import numpy as np
import pickle
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