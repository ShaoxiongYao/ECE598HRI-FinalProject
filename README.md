# ECE598 HRI Final Project

realsense camera serial number:
+ left: f0220315
+ right: f0271386
+ torso: f0190400

### Record RGB and depth images from multiple cameras

start ros node using command:

    roslaunch realsense2_camera rs_camera.launch serial_no:=f0190400 camera:=cam_topic enable_sync:=True 
    align_depth:=True color_width:=1920 color_height:=1080 color_fps:=30 depth_width:=1920 depth_height:=1080 depth_fps:=30

record rosbag using commnd:

    rosbag record cam_topic1 cam_topic2

Useful topics:

    /camera/aligned_depth_to_color/image_raw
    /camera/color/image_raw

    /cam_topic1/color/image_raw /cam_topic2/color/image_raw /cam_topic3/color/image_raw /cam_topic1/aligned_depth_to_color/image_raw /cam_topic2/aligned_depth_to_color/image_raw /cam_topic3/aligned_depth_to_color/image_raw

