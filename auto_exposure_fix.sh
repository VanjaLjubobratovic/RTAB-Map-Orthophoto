#!/bin/bash

#This script waits for /camera/camera node to come online
#and sets depth module's autoexposure to True because
#camera drops out of autoexposure mode somewhere during startup
#even though ros2 param get returns true

camera_node_name="/camera/camera"
parameter_name="depth_module.enable_auto_exposure"

while true; do
    # Check the current parameter value
    current_value=$(ros2 param get $camera_node_name $parameter_name 2>/dev/null)
    
    if [ "$current_value" == "Boolean value is: True" ] || [ "$current_value" == "Boolean value is: False" ]; then
        # Node is up, setting autoexposure to True
        sleep 0.2
        ros2 param set $camera_node_name $parameter_name "True"
        echo "Param set to True"
        break
    else
        echo "Param not yet set"
    fi

    # Sleep for a while and check again
    sleep 0.5
done

exit
