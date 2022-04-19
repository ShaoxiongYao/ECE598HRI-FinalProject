# ECE598 HRI Final Project

realsense camera serial number:
+ left: f0220315
+ torso: f0190400

### Record RGB and depth images from multiple cameras

    start ros node using command:

        roslaunch realsense2_camera rs_camera.launch serial_no:=f0190400 camera:=cam_topic
    
    record rosbag using commnd:
    
        rosbag record cam_topic1 cam_topic2