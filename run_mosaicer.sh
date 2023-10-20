#!/bin/bash

ros2 launch mosaicing_pkg mosaicer.launch.py \
cloud_decimation:=1 \
cloud_voxel_size:=0.01 \
cloud_max_depth:=3.5 \
cloud_min_depth:=0.5 \
num_threads:=8 \
grid_resolution:=0.008 \
interpolate:=false \
interpolation_method:="NN" \
show_live:=true
