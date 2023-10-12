#!/bin/bash

# This script waits for all camera nodes to come online
# and sets depth module's autoexposure to True because
# camera drops out of autoexposure mode somewhere during startup
# even though ros2 param get returns true

# Get a list of all ros2 nodes containing "camera" in their name
camera_nodes=

wait_time=0.0

# Keep checking the node list until at least one node with "camera" is found
while (( $(bc <<< "$wait_time <= 10.0") )); do
    camera_nodes=$(ros2 node list | grep "camera")
    if [ -z "$camera_nodes" ]; then
        echo "No camera nodes found. Waiting..."
        wait_time=$(bc <<< "$wait_time + 0.25")
        sleep 0.25
    else
        break  # Exit the loop when at least one camera node is found
    fi
done

parameter_name="depth_module.enable_auto_exposure"

for camera_node_name in $camera_nodes; do
    while true; do
        # If node is not yet up this command will return "Parameter not set"
        current_value=$(ros2 param get $camera_node_name $parameter_name 2>/dev/null)

        if [ "$current_value" == "Boolean value is: True" ] || [ "$current_value" == "Boolean value is: False" ]; then
            ros2 param set $camera_node_name $parameter_name "True"
            echo "Param set to True for node: $camera_node_name"
            break
        else
            echo "Param not yet set for node: $camera_node_name"
        fi

        # Sleep for a while and check again
        sleep 0.25
    done
done

exit

