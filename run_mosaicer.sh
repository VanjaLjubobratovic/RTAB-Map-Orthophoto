#!/bin/bash

ros2 launch mosaicing_pkg mosaicer.launch.py \
cloud_decimation:=1 \
cloud_voxel_size:=0.01 \
cloud_max_depth:=3.5 \
interpolate:=false \
show_live:=true
