#!/bin/bash

# Without this ros2 launch won't be able to locate launchfiles
# in ros2_ws folder
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
. install/local_setup.bash

ros2 launch realsense2_camera rs_launch.py \
rgb_camera.profile:=848x480x30 \
depth_module.profile:=848x480x30 \
enable_gyro:=true \
enable_accel:=true \
unite_imu_method:=1 \
gyro_fps:=400 \
accel_fps:=250 \
enable_infra1:=false \
enable_infra2:=false \
enable_sync:=false \
align_depth.enable:=true \
pointcloud.enable:=false \
initial_reset:=true