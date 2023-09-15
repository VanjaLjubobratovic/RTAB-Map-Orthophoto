#!/bin/bash
source /opt/ros/humble/setup.bash

LC_NUMERIC="en_US.UTF-8"; ros2 launch rtabmap_launch rtabmap.launch.py \
rtabmap_args:="--delete_db_on_start" \
rviz:=false \
rtabmapviz:=true \
frame_id:=camera_link \
rgb_topic:=/camera/color/image_raw \
depth_topic:=/camera/aligned_depth_to_color/image_raw \
camera_info_topic:=/camera/color/camera_info \
approx_sync:=false \
wait_imu_to_init:=true \
queue_size:=300 \
cfg:=/home/vanja/RTAB-Map-Orthophoto/configs/default_config.ini\
