#!/bin/bash

timestamp=$(date +%Y-%m-%d_%H-%M)

filename=$timestamp

cd ~/camera_bags

ros2 bag record -o $filename \
/camera/aligned_depth_to_color/camera_info \
/camera/aligned_depth_to_color/image_raw \
/camera/color/camera_info \
/camera/color/image_raw \
/camera/imu \
/camera/imu_info \
/tf_static \
/tf
