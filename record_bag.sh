#!/bin/bash

source /opt/ros/humble/setup.bash
cd ~/camera_bags

ros2 bag record -o newest_camera \
/camera/aligned_depth_to_color/camera_info \
/camera/aligned_depth_to_color/image_raw \
/camera/color/camera_info \
/camera/color/image_raw \
/camera/imu