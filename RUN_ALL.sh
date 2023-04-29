#!/bin/bash

MAPPING=false
RECORD=false
PLAY=false

declare -a scripts=("run_camera" "run_imu_filter")
declare -a titles=("Camera" "Imu_filter")

while [[ $# -gt 0 ]]
do
    key="$1"

    case $key in
        -h|--help)
        echo "Usage: $0 [-r|--record] [-m|--mapping] [-p|--play] [-h]"
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
        -p|--play)
        PLAY=true
        shift # past argument
        ;;
        *)    # unknown option
        echo "Unknown option: $key"
        exit 1
        ;;
    esac
done

# For readability rather than in loop above
if $MAPPING; then
  scripts+=("run_rtabmap")
  titles+=("RTAB_Map")
fi

if $RECORD; then
  scripts+=("record_bag")
  titles+=("Bag_record")
fi

# Can't run camera when replay is on
if $PLAY; then
  scripts+=("replay_bag")
  unset scripts[0]
  titles+=("Bag_replay")
  unset titles[0]
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
