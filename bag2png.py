import subprocess
import yaml
import rosbag
import cv2
import os
from cv_bridge import CvBridge
import numpy as np
from pathlib import Path

ROOT_DIR = 'Dataset'
BAG_NAME = '2022-04-22-15-59-07'

save_imgs = False

if __name__ == '__main__':
    for cam_topic in ['cam_left','cam_right','cam_torso']:
        cam_name = cam_topic
        FILENAME = os.path.join(ROOT_DIR, 'rosbags', BAG_NAME+'.bag')
        bag = rosbag.Bag(FILENAME)
        for i in range(2):
            if (i == 0):
                TOPIC = f'/{cam_topic}/aligned_depth_to_color/image_raw'
                DESCRIPTION = 'depth_'
            else:
                TOPIC = f'/{cam_topic}/color/image_raw'
                DESCRIPTION = 'color_'
            image_topic = bag.read_messages(TOPIC)

            timestamp_lst = []
            for k, b in enumerate(image_topic):

                if save_imgs:
                    bridge = CvBridge()
                    cv_image = bridge.imgmsg_to_cv2(b.message, b.message.encoding)
                    cv_image.astype(np.uint8)

                    img_dir =  f'{ROOT_DIR}/{BAG_NAME}/{cam_name}/{DESCRIPTION[:-1]}/'
                    Path(img_dir).mkdir(parents=True, exist_ok=True)
                    img_fn = DESCRIPTION + str(b.timestamp) + '.png'

                    img_path = os.path.join(img_dir, img_fn)
                    if (DESCRIPTION == 'depth_'):
                        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(cv_image, alpha=0.03), cv2.COLORMAP_JET)
                        cv2.imwrite(img_path, cv_image)
                    else:
                        img_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
                        cv2.imwrite(img_path, img_rgb)
                    print("image path:", img_path)
                    print('saved: ' + img_fn)

                timestamp_lst.append(b.timestamp.to_sec())

            if timestamp_lst != []:
                print("start time:", min(timestamp_lst))
                print("end time:", max(timestamp_lst))
                timestamp_diff = np.diff(timestamp_lst)
                print("average frequency:", 1/np.mean(timestamp_diff))

    bag.close()

    print('PROCESS COMPLETE')