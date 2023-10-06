#!/bin/bash

ros2 launch mosaicing_pkg multicam.launch.py \
pointcloud.enable1:=true \
pointcloud.enable2:=true \
serial_no1:=_203522250780 \
serial_no2:=_043422251337 \
align_depth.enable1:=true \
align_depth.enable2:=true \
initial_reset1:=true \
initial_reset2:=true \
depth_module.enable_auto_exposure1:=true \
depth_module.enable_auto_exposure2:=true \
rgb_camera.profile1:=848x480x30 \
rgb_camera.profile2:=848x480x30 \
depth_module.profile1:=848x480x30 \
depth_module.profile2:=848x480x30 \
decimation_filter.enable1:=true \
decimation_filter.enable2:=true \
temporal_filter.enable1:=true \
temporal_filter.enable2:=true