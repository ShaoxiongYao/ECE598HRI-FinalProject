import subprocess
import yaml
import rosbag
import cv2
from cv_bridge import CvBridge
import numpy as np


FILENAME = 'Dataset/rosbags/2022-04-19-11-03-27.bag'
ROOT_DIR = 'Dataset'

cam_name = 'cam_left'
save_imgs = False

if __name__ == '__main__':
    bag = rosbag.Bag(FILENAME)
    for i in range(2):
        if (i == 0):
            TOPIC = f'/{cam_name}/depth/image_rect_raw'
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
                    cv2.imwrite(ROOT_DIR + '/depth/' + DESCRIPTION + str(b.timestamp) + '.png', cv_image)
                else:
                    cv2.imwrite(ROOT_DIR + '/color/' + DESCRIPTION + str(b.timestamp) + '.png', cv_image)
                print('saved: ' + DESCRIPTION + str(b.timestamp) + '.png')

            timestamp_lst.append(b.timestamp.to_sec())

        if timestamp_lst != []:
            print("start time:", min(timestamp_lst))
            print("end time:", max(timestamp_lst))
            timestamp_diff = np.diff(timestamp_lst)
            print("average frequency:", 1/np.mean(timestamp_diff))

    bag.close()

    print('PROCESS COMPLETE')