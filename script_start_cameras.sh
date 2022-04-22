#!/bin/bash
set -Eeuo pipefail
set -x

(trap 'kill 0' SIGINT;\

roslaunch realsense2_camera rs_camera.launch serial_no:=f0220315 camera:=cam_left align_depth:=True enable_sync:=True &\

sleep 5
roslaunch realsense2_camera rs_camera.launch serial_no:=f0271386 camera:=cam_right align_depth:=True enable_sync:=True &\

sleep 5
roslaunch realsense2_camera rs_camera.launch serial_no:=f0190400 camera:=cam_torso align_depth:=True enable_sync:=True)

export ROS_MASTER_URI=http://localhost:11311
