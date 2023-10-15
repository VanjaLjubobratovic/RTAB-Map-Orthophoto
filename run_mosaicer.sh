#!/bin/bash

ros2 launch mosaicing_pkg mosaicer.launch.py \
cloud_decimation:=1 \
cloud_voxel_size:=0.01 \
cloud_max_depth:=4.0 \
num_threads:=8 \
grid_resolution:=0.005 \
interpolate:=true \
interpolation_method:="NN" \
show_live:=true
