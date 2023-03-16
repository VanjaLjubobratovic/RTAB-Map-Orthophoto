#!/bin/bash
source /opt/ros/humble/setup.bash

ros2 launch rtabmap_ros rtabmap.launch.py \
rviz:=false `#Not needed for now, rtabmapviz suddenly works`\
rtabmapviz:=true \
frame_id:=camera_link \
rgb_topic:=/camera/color/image_raw \
depth_topic:=/camera/aligned_depth_to_color/image_raw \
camera_info_topic:=/camera/color/camera_info \
approx_sync:=false \
wait_imu_to_init:=true \
queue_size:=100 \