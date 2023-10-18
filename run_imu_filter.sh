#! /bin/bash

# This runs filter for camera IMU
# Receives raw messages on /camera/imu (/imu/data_raw by default) and
# publishes on /imu/data

# 2 options include: Madgwick and Complementary filter

#<<COMMENT
ros2 run imu_complementary_filter complementary_filter_node \
--ros-args --remap /imu/data_raw:=/camera/imu \
--param fixed_frame:=odom
#COMMENT

<<COMMENT
ros2 run imu_filter_madgwick imu_filter_madgwick_node \
--ros-args --remap /imu/data_raw:=/camera/imu \
--param use_mag:=false \
--param publish_tf:=false
COMMENT
