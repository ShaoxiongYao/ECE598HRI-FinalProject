import numpy as np
import open3d as o3d
import time
import copy

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp],
                                      zoom=0.4559,
                                      front=[0.6452, -0.3036, -0.7011],
                                      lookat=[1.9892, 2.0208, 1.8945],
                                      up=[-0.2779, -0.9482, 0.1556])

def preprocess_point_cloud(pcd, voxel_size):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh

def execute_fast_global_registration(source_down, target_down, source_fpfh,
                                     target_fpfh, voxel_size):
    distance_threshold = voxel_size * 0.5
    print(":: Apply fast global registration with distance threshold %.3f" \
            % distance_threshold)
    result = o3d.pipelines.registration.registration_fast_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh,
        o3d.pipelines.registration.FastGlobalRegistrationOption(
            maximum_correspondence_distance=distance_threshold))
    return result

def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    print(":: RANSAC registration on downsampled point clouds.")
    print("   Since the downsampling voxel size is %.3f," % voxel_size)
    print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        3, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))
    return result

def color_registration(source, target, iter, radius):

    current_transformation = np.identity(4)
    print("3-1. Downsample with a voxel size %.2f" % radius)
    source_down = source.voxel_down_sample(radius)
    target_down = target.voxel_down_sample(radius)

    print("3-2. Estimate normal.")
    source_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))
    target_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))

    print("3-3. Applying colored point cloud registration")
    result_icp = o3d.pipelines.registration.registration_colored_icp(
        source_down, target_down, radius, current_transformation,
        o3d.pipelines.registration.TransformationEstimationForColoredICP(),
        o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=1e-6,
                                                          relative_rmse=1e-6,
                                                          max_iteration=iter))
    current_transformation = result_icp.transformation
    print(result_icp)
    return result_icp

def display_inlier_outlier(cloud, ind):
    inlier_cloud = cloud.select_by_index(ind)
    outlier_cloud = cloud.select_by_index(ind, invert=True)

    print("Showing outliers (red) and inliers (gray): ")
    outlier_cloud.paint_uniform_color([1, 0, 0])
    inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud],
                                      zoom=0.3412,
                                      front=[0.4257, -0.2125, -0.8795],
                                      lookat=[2.6172, 2.0475, 1.532],
                                      up=[-0.0694, -0.9768, 0.2024])

def pick_points(pcd):
    print("")
    print(
        "1) Please pick at least three correspondences using [shift + left click]"
    )
    print("   Press [shift + right click] to undo point picking")
    print("2) After picking points, press 'Q' to close the window")
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()  # user picks points
    vis.destroy_window()
    print("")
    return vis.get_picked_points()

def interactive_registration(source, target, picked_id_source, picked_id_target, 
                             transformation_name=None):
    assert (len(picked_id_source) >= 3 and len(picked_id_target) >= 3)
    assert (len(picked_id_source) == len(picked_id_target))
    corr = np.zeros((len(picked_id_source), 2))
    corr[:, 0] = picked_id_source
    corr[:, 1] = picked_id_target

    # estimate rough transformation using correspondences
    print("Compute a rough transform using the correspondences given by user")
    p2p = o3d.pipelines.registration.TransformationEstimationPointToPoint()
    trans_init = p2p.compute_transformation(source, target,
                                            o3d.utility.Vector2iVector(corr))

    # point-to-point ICP for refinement
    print("Perform point-to-point ICP refinement")
    threshold = 0.03  # 3cm distance threshold
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source, target, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint())
    draw_registration_result(source, target, reg_p2p.transformation)
    print("registration result:", reg_p2p.transformation)
    if transformation_name is not None:
        np.save(transformation_name, reg_p2p.transformation)
    return reg_p2p

if __name__ == '__main__':

    pcd_dir = 'Calibration/data/point_cloud'
    keys_list = ['realsense_left','realsense_right','realsense_torso']
    pcd_dict = { k:o3d.geometry.PointCloud() for k in keys_list }
    for key in keys_list:
        pcd_fn = pcd_dir+'/'+key+'.npy'
        pcd_ary = np.load(pcd_fn)
        pcd_dict[key].points = o3d.utility.Vector3dVector(pcd_ary[:, :3])
        pcd_dict[key].colors = o3d.utility.Vector3dVector(pcd_ary[:, 3:])
    
    o3d.visualization.draw_geometries([pcd for k, pcd in pcd_dict.items()])

    # remove outliers from the original point cloud is time-consuming
    # for k, pcd in pcd_dict.items():
    #     cl, ind = pcd.remove_statistical_outlier(nb_neighbors=20,
    #                                               std_ratio=2.0)
    #     display_inlier_outlier(pcd, ind)
    #     pcd_dict[k] = pcd.select_by_index(ind)

    # manual registration
    voxel_size = 0.03
    for k, pcd in pcd_dict.items():
        pcd = pcd.voxel_down_sample(voxel_size)
        cl, ind = pcd.remove_statistical_outlier(nb_neighbors=20,
                                                 std_ratio=2.0)
        display_inlier_outlier(pcd, ind)
        pcd_dict[k] = pcd.select_by_index(ind)
    
    trans_dir = 'Calibration/data/extrinsics'

    # left source, right target
    # [[ 0.16269235, -0.25042581,  0.95436791, -2.00892072],                              
    # [ 0.19965646,  0.95560061,  0.21671357, -0.21819983],                                                   
    # [-0.96626523,  0.15528807,  0.20546804,  1.83963049],                                                   
    # [ 0.         , 0. ,         0.,          1.        ]]
    # picked_left  = pick_points(pcd_dict['realsense_left'])
    # picked_right = pick_points(pcd_dict['realsense_right'])
    # interactive_registration(pcd_dict['realsense_left'], 
    #                          pcd_dict['realsense_right'], 
    #                          picked_left, picked_right, 
    #                          trans_dir+'/left2right.npy')

    picked_left  = pick_points(pcd_dict['realsense_left'])
    picked_torso = pick_points(pcd_dict['realsense_torso'])
    interactive_registration(pcd_dict['realsense_left'], 
                             pcd_dict['realsense_torso'], 
                             picked_left, picked_torso,
                             trans_dir+'/left2torso.npy')

    picked_right  = pick_points(pcd_dict['realsense_right'])
    picked_torso = pick_points(pcd_dict['realsense_torso'])
    interactive_registration(pcd_dict['realsense_right'], 
                             pcd_dict['realsense_torso'], 
                             picked_right, picked_torso,
                             trans_dir+'/right2torso.npy')

    # global registration
    # start = time.time()
    # result_fast = execute_global_registration(pcd_left, pcd_right,
    #                                           pcd_left_fpfh, pcd_right_fpfh,
    #                                           voxel_size)
    # print("Global registration took %.3f sec.\n" % (time.time() - start))
    # print(result_fast)
    # draw_registration_result(pcd_left, pcd_right, result_fast.transformation)
    
    # color icp
    # pcd_left = pcd_dict['realsense_left']
    # pcd_right = pcd_dict['realsense_right']

    # results = color_registration(pcd_left, pcd_right, 50, 0.04)
    # draw_registration_result(pcd_left, pcd_right, results.transformation)