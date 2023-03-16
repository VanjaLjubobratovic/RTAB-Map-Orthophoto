#! /bin/bash

# This runs filter for camera IMU
# Receives raw messages on /camera/imu (/imu/data_raw by default) and
# publishes on /imu/data

# 2 options include: Madgwick and Complementary filter

ros2 run imu_complementary_filter complementary_filter_node \
--ros-args --remap /imu/data_raw:=/camera/imu 