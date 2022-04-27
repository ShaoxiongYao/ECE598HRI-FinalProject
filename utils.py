from glob import glob
import pandas as pd
from tqdm import tqdm
import numpy as np
import cv2 as cv
import cv2
from create_point_cloud import load_whole_point_cloud
import scipy


def get_proper_image_paths(unfilled_df,datasets_parent_folder):
    filled_df = unfilled_df.copy()
    for col in unfilled_df.columns:
        if(col.startswith('cam')):
            filled_df.loc[:,col] = unfilled_df.loc[:,col].str.replace('{}',datasets_parent_folder)
    return filled_df

def strip_dataset_parent_folder(filled_df,parent_folder_name):
    # post-processing the dataset:
    reusable_df = filled_df.copy()
    for col in reusable_df.columns:
        if(col.startswith('cam')):
            tmp1 = reusable_df[col].str.split(parent_folder_name,expand = True).loc[:,1]
            tmp1 = '{}/' + tmp1
            reusable_df.loc[:,col] = tmp1
    return reusable_df

def get_aligned_dataset(dataset_folder,master_camera = 'cam_torso_depth'):
    cam_folders = sorted(glob(dataset_folder+'/*'))

    dataset_dict = {}

    for cam_folder in cam_folders:
        cam_name = cam_folder.split('/')[-1]

        color_dataset = sorted(glob('{}/color/*'.format(cam_folder)))
        depth_dataset = sorted(glob('{}/depth/*'.format(cam_folder)))

        dataset_dict.update({'{}_depth'.format(cam_name):depth_dataset,'{}_color'.format(cam_name):color_dataset})
        
        dataframes = {}
        for i in dataset_dict.keys():
            df = pd.DataFrame({'frame':dataset_dict[i]})
            timestamps = df.frame.str.split('_',expand = True).iloc[:,-1].str.split('.',expand = True).iloc[:,0].astype(float)
            df['timestamps'] = timestamps
            df.columns = [i,'timestamps']
            dataframes.update({i:df})
            
    dict_keys = list(dataset_dict.keys())

    original_df = dataframes[master_camera]
    for i in dict_keys:
        if(i != master_camera):
            original_df[i] = np.nan

    sorted_df = original_df.sort_values(by = 'timestamps')
    filled_df = sorted_df.fillna(method= 'ffill',limit = 11)

    for i in tqdm(range(original_df.shape[0])):
        timestamp = original_df.timestamps[i]
        for j in dict_keys:
            if(j != master_camera):
                td = np.min(np.abs(dataframes[j].timestamps-timestamp))/1000000000
                index = np.argmin(np.abs(dataframes[j].timestamps-timestamp))
                if(td < 1/60):
                    original_df.loc[i,j] = dataframes[j].loc[index,j]

    original_df.shape

    clean_df = original_df.dropna()
    clean_df.reset_index(drop = True, inplace = True)
    return clean_df

def find_hand_center(hand_points,images_df,img_index,frame,cam_side = 'right',trans_dir = './Calibration/data/extrinsics'):
    try:
        # create grayscale image with white circle (255) on black background (0)
        mask = np.zeros(shape = frame.shape[:2],dtype = np.uint8)
        hand_points = hand_points[hand_points[:,2] > 0.1]
        for point in hand_points:
            cv2.circle(mask,(int(point[0]),int(point[1])),3,255,-1)
        coordy,coordx = np.where(mask>0)
        points = np.ravel_multi_index((coordy,coordx),frame.shape[:2])

        color = images_df.loc[img_index,'cam_{}_color'.format(cam_side)]
        depth = images_df.loc[img_index,'cam_{}_depth'.format(cam_side)]
        pcd = load_whole_point_cloud(color,depth,'realsense_{}'.format(cam_side))
        hand = pcd.select_by_index(points)
        hand.remove_non_finite_points()
        E2torso = np.load(trans_dir+f'/right2torso.npy')
        result = hand.cluster_dbscan(0.1, 20)
        mode = scipy.stats.mode(result)
        actual_hand_index = np.where(result == mode.mode[0])[0]
        actual_hand = hand.select_by_index(actual_hand_index)
        flipped_hand = actual_hand.transform(E2torso)
        positions = np.asarray(flipped_hand.points)
        return positions.mean(axis = 0)
    except Exception as e:
        print('missing data due to {}\n'.format(e))
        return None