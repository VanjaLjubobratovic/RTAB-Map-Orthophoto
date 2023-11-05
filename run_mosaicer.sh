#!/bin/bash

ros2 launch mosaicing_pkg mosaicer.launch.py \
cloud_decimation:=1 \
cloud_voxel_size:=0.008 \
cloud_max_depth:=4.0 \
cloud_min_depth:=0.5 \
num_threads:=8 \
grid_resolution:=0.008 \
sor_filter_enable:=true \
sor_neighbors:=40 \
sor_stdev_mul:=1.0 \
dist_filter_enable:=true \
dist_stedv_mul:=1.0 \
interpolate:=false \
interpolation_method:="NN" \
show_live:=true
