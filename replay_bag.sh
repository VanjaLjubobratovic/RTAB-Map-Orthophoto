#!/bin/bash

source /opt/ros/humble/setup.bash
cd ~/camera_bags

read -p "Specify bag filename: ~/camera_bags/" -e filename

ros2 bag play $filename --start-paused