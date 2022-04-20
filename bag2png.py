import subprocess
import yaml
import rosbag
import cv2
from cv_bridge import CvBridge
import numpy as np

cam_id = 'cam_right'

FILENAME = f'Dataset/rosbags/2022-04-19-20-27-48.bag'
ROOT_DIR = 'Dataset'

cam_name = 'cam_topic1'
save_imgs = True

if __name__ == '__main__':
    bag = rosbag.Bag(FILENAME)
    for i in range(2):
        if (i == 0):
            TOPIC = f'/{cam_name}/aligned_depth_to_color/image_raw'
            DESCRIPTION = 'depth_'
        else:
            TOPIC = f'/{cam_name}/color/image_raw'
            DESCRIPTION = 'color_'
        image_topic = bag.read_messages(TOPIC)

        timestamp_lst = []
        for k, b in enumerate(image_topic):

            if save_imgs:
                bridge = CvBridge()
                cv_image = bridge.imgmsg_to_cv2(b.message, b.message.encoding)
                cv_image.astype(np.uint8)
                if (DESCRIPTION == 'depth_'):
                    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(cv_image, alpha=0.03), cv2.COLORMAP_JET)
                    fn = ROOT_DIR + f'/{cam_id}_imgs/depth/' + DESCRIPTION + str(b.timestamp) + '.png'
                    cv2.imwrite(fn, cv_image)
                else:
                    fn = ROOT_DIR + f'/{cam_id}_imgs/color/' + DESCRIPTION + str(b.timestamp) + '.png'
                    cv2.imwrite(fn, cv_image)
                print("fn:", fn)
                print('saved: ' + DESCRIPTION + str(b.timestamp) + '.png')

            timestamp_lst.append(b.timestamp.to_sec())

        if timestamp_lst != []:
            print("start time:", min(timestamp_lst))
            print("end time:", max(timestamp_lst))
            timestamp_diff = np.diff(timestamp_lst)
            print("average frequency:", 1/np.mean(timestamp_diff))

    bag.close()

    print('PROCESS COMPLETE')