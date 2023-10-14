#!/bin/bash

resolution="1280x720x60"

LC_NUMERIC="en_US.UTF-8"; ros2 launch realsense2_camera rs_launch.py \
rgb_camera.profile:=$resolution \
depth_module.profile:=$resolution \
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
initial_reset:=true \
depth_module.enable_auto_exposure:=true \
decimation_filter.enable:=true \
temporal_filter.enable:=true \
spatial_filter.enable:=true \