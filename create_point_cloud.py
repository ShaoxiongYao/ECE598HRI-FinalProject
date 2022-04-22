import open3d as o3d
from calibration_utils import load_cam_K, get_image_names

def load_point_cloud(color_path, depth_path, cam_key):
    color_raw = o3d.io.read_image(color_path)
    depth_raw = o3d.io.read_image(depth_path)

    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color_raw, depth_raw, 
                                                                    depth_trunc=10.0,
                                                                    convert_rgb_to_intensity=False)

    K_fn = 'intrinsic_calibrations.p'
    cam_K_dict = load_cam_K(K_fn, 1280, 720)

    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, cam_K_dict[cam_key], 
                                                         project_valid_depth_only=False)
    pcd.remove_non_finite_points()
    return pcd

if __name__ == '__main__':

    img_fn_dict = get_image_names('Dataset/2022-04-22-15-35-26')

    left_color_path = img_fn_dict['cam_left']['color_fn_lst'][0]
    left_depth_path = img_fn_dict['cam_left']['depth_fn_lst'][0]
    left_pcd = load_point_cloud(left_color_path, left_depth_path, 'realsense_left')
    o3d.io.write_point_cloud('Calibration/data/point_cloud/left.ply', left_pcd)

    right_color_path = img_fn_dict['cam_right']['color_fn_lst'][0]
    right_depth_path = img_fn_dict['cam_right']['depth_fn_lst'][0]
    right_pcd = load_point_cloud(right_color_path, right_depth_path, 'realsense_right')
    o3d.io.write_point_cloud('Calibration/data/point_cloud/right.ply', right_pcd)

    torso_color_path = img_fn_dict['cam_torso']['color_fn_lst'][0]
    torso_depth_path = img_fn_dict['cam_torso']['depth_fn_lst'][0]
    torso_pcd = load_point_cloud(torso_color_path, torso_depth_path, 'realsense_torso')
    o3d.io.write_point_cloud('Calibration/data/point_cloud/torso.ply', torso_pcd)
