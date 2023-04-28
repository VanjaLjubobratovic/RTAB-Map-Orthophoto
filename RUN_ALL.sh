#!/bin/bash

MAPPING=false
RECORD=false

declare -a scripts=("run_camera" "run_imu_filter")
declare -a titles=("Camera" "Imu_filter")

while [[ $# -gt 0 ]]
do
    key="$1"

    case $key in
        -h|--help)
        echo "Usage: $0 [-r|--record] [-m|--mapping] [-h]"
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
        *)    # unknown option
        echo "Unknown option: $key"
        exit 1
        ;;
    esac
done

#! For readability rather than in loop above
if $MAPPING; then
  scripts+=("run_rtabmap")
  titles+=("RTAB_Map")
fi

if $RECORD; then
  scripts+=("record_bag")
  titles+=("Bag_record")
fi

# Building command to open all specified scripts in options
cmd="gnome-terminal"
for (( i=0; i<${#titles[@]}; i++ )); do
  cmd+=" --tab --title=\"${titles[$i]}\" -e \"bash -c 'cd $PWD; ./${scripts[$i]}.sh; $SHELL'\""
done

eval "${cmd}"
