#!/bin/bash

timestamp=$(date +%Y-%m-%d_%H-%M)

read -p "Specify bag filename: " filename

if [ -z "$filename" ]; then
	filename="camera_recording"
fi

filename+="_"$timestamp


source /opt/ros/humble/setup.bash
cd ~/camera_bags

ros2 bag record --start-paused -o $filename \
/camera/aligned_depth_to_color/camera_info \
/camera/aligned_depth_to_color/image_raw \
/camera/color/camera_info \
/camera/color/image_raw \
/camera/imu \
/camera/imu_info \
/tf_static \
/tf