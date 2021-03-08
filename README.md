# vid_rectify

Ros Node for rectifying Stereo images.

## Pre-requisites
1. Robot Operating System (ROS)
2. OpenCV

## How to use?
1. Clone the repo inside the ros workspace (catkin workspace)

2. build the package.
```
$ cd catkin_ws

OR

$ cd <ROS Workspace>

$ catkin_make --pkg vid_rectify
```
3. Make sure the stereo topics to be rectified are up and running.

4. Launch the vid_rectify node.
```
$ roslaunch vid_rectify vid_rectify.launch 
```
By default the argument values of the launch file are,
* stereo_param -> yaml/m210_stereo_param.yaml
* left_stereo_topic -> /dji_osdk_ros/stereo_vga_front_left_images
* right_stereo_topic -> /dji_osdk_ros/stereo_vga_front_right_images
* display -> false

To specify these arguments with different values to use, use the following command:
```
$ roslaunch vid_rectify vid_rectify.launch stereo_param:=<absolute_path_to_file> left_stereo_topic:=<left_topic_name> right_stereo_topic:=<right_topic_name> display:=<true or false>
```