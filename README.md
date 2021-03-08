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
$ roslaunch vid_rectify vid_rectify_640x480.launch
or
$ roslaunch vid_rectify vid_rectify_320x240.launch 
```
By default the argument values of the launch file are,
640x480 resolution:
* stereo_param -> yaml/m210_stereo_param_640x480.yaml
* left_stereo_topic -> /dji_osdk_ros/stereo_vga_front_left_images
* right_stereo_topic -> /dji_osdk_ros/stereo_vga_front_right_images
* display -> false

320x240 resolution:
* stereo_param -> yaml/m210_stereo_param_320x240.yaml
* left_stereo_topic -> /dji_osdk_ros/stereo_240p_front_left_images
* right_stereo_topic -> /dji_osdk_ros/stereo_240p_front_right_images
* display -> false

### Optional
To specify these arguments with different file_path and topic names to use, use the following command:
```
$ roslaunch vid_rectify vid_rectify_<resolution>.launch stereo_param:=<absolute_path_to_file> left_stereo_topic:=<left_topic_name> right_stereo_topic:=<right_topic_name> display:=<true or false>
```