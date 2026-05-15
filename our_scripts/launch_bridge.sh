#!/bin/bash
# Launch server bridge

# Source ROS 2 and workspace
source /opt/ros/humble/setup.bash
source  /home/developer/workspace/ros2_ws/install/setup.bash

# Force ROS_DOMAIN_ID to match other terminals
export ROS_DOMAIN_ID = 0

echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "Starting brdige_node..."
echo ""

ros2 run ouranos_bridge bridge_node