#!/bin/bash
set -Eeuo pipefail
set -x

rosbag record /cam_left/color/image_raw /cam_right/color/image_raw /cam_torso/color/image_raw\
              /cam_left/aligned_depth_to_color/image_raw\
              /cam_right/aligned_depth_to_color/image_raw\
              /cam_torso/aligned_depth_to_color/image_raw
