#!/bin/bash

CAMERA=true
MAPPING=false
RECORD=false
PLAY=false
ORTHOPHOTO=false

declare -a scripts
declare -a titles

while [[ $# -gt 0 ]]
do
    key="$1"

    case $key in
        -h|--help)
        echo "Usage: $0 [-r|--record] [-m|--mapping] [-p|--play] [-o|--orthophoto] [-h|--help]"
        exit 0
        ;;
        -r|--record)
        RECORD=true
        shift # past argument
        ;;
        -m|--mapping)
        MAPPING=true
        shift # past argument
        ;;
        -o|--orthophoto)
        ORTHOPHOTO=true
        shift # past argument
        ;;
        -p|--play)
        PLAY=true
        CAMERA=false
        shift # past argument
        ;;
        *)    # unknown option
        echo "Unknown option: $key"
        exit 1
        ;;
    esac
done

if $CAMERA; then
  scripts+=("run_camera" "auto_exposure_fix" "run_imu_filter")
  titles+=("RealSense camera node" "Autoexposure Fix" "Imu_filter")
fi

# For readability rather than in loop above
if $MAPPING; then
  scripts+=("run_rtabmap")
  titles+=("RTAB_Map")
fi

if $ORTHOPHOTO && $MAPPING; then
  scripts+=("run_mosaicer")
  titles+=("Mosaicer")
fi

if $RECORD; then
  scripts+=("record_bag")
  titles+=("Bag_record")
fi

# Can't run camera when replay is on
if $PLAY; then
  scripts+=("replay_bag")
  titles+=("Bag_replay")
fi

if $PLAY && $RECORD; then
  echo "Can't record and play bag at the same time!"
  exit 1
fi

# Building command to open all specified scripts in options
cmd="gnome-terminal"
for (( i=0; i<=${#titles[@]}; i++ )); do
  if [[ ! -v titles[$i] ]]; then
    continue
  fi
  cmd+=" --tab --title=\"${titles[$i]}\" -e \"bash -c 'cd $PWD; ./${scripts[$i]}.sh; $SHELL'\""
done

echo $cmd

eval "${cmd}"
