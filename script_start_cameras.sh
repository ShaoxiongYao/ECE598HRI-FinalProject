#!/bin/bash
set -Eeuo pipefail
set -x

(trap 'kill 0' SIGINT;\

roslaunch realsense2_camera rs_camera.launch serial_no:=f0220315 camera:=cam_left align_depth:=True color_width:=1920 color_height:=1080 color_fps:=30 depth_width:=1920 depth_height:=1080 depth_fps:=30 &\

sleep 5
roslaunch realsense2_camera rs_camera.launch serial_no:=f0271386 camera:=cam_right align_depth:=True color_width:=1920 color_height:=1080 color_fps:=30 depth_width:=1920 depth_height:=1080 depth_fps:=30 &\

sleep 5
roslaunch realsense2_camera rs_camera.launch serial_no:=f0190400 camera:=cam_torso align_depth:=True color_width:=1920 color_height:=1080 color_fps:=30 depth_width:=1920 depth_height:=1080 depth_fps:=30)
