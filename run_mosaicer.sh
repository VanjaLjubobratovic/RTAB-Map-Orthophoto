#!/bin/bash

ros2 launch mosaicing_pkg mosaicer.launch.py \
cloud_decimation:=2 \
interpolate:=false \
show_live:=true