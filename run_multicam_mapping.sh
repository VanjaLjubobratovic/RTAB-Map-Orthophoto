#!/bin/bash

# Running the script which fixes auto exposure for both camera nodes
gnome-terminal --tab --title="Autoexposure Fix" -e "bash -c 'cd $PWD; ./auto_exposure_fix.sh; $SHELL'"

ros2 launch mosaicing_pkg multicam.launch.py \
fixed_frame:=base_link \
camera_distance:=0.57 \
pointcloud.enable1:=true \
pointcloud.enable2:=true \
serial_no1:=_203522250780 \
serial_no2:=_043422251337 \
align_depth.enable1:=true \
align_depth.enable2:=true \
unite_imu_method1:=1 \
unite_imu_method2:=1 \
enable_gyro1:=true \
enable_gyro2:=true \
enable_accel1:=true \
enable_accel2:=true \
initial_reset1:=true \
initial_reset2:=true \
depth_module.enable_auto_exposure1:=true \
depth_module.enable_auto_exposure2:=true \
rgb_camera.profile1:=848x480x60 \
rgb_camera.profile2:=848x480x60 \
depth_module.profile1:=848x480x60 \
depth_module.profile2:=848x480x60 \
decimation_filter.enable1:=true \
decimation_filter.enable2:=true \
temporal_filter.enable1:=false \
temporal_filter.enable2:=false \
spatial_filter.enable1:=true \
spatial_filter.enable2:=true \
wait_imu_to_init:=false \
cfg:=/home/vanja/RTAB-Map-Orthophoto/configs/rtab_multi.ini \
cameras_enabled:=true \
tf_publisher_enabled:=true \
rtab_enabled:=true \
bag_record:=false 
