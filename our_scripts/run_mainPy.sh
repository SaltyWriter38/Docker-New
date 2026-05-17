#!/bin/bash

source /opt/ros/humble/setup.bash
source /home/developer/workspace/ros2_ws/install/setup.bash

export ROS_DOMAIN_ID=0

echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "Starting main.py..."
echo ""

python3 /home/developer/workspace/ros2_ws/src/gpig/gpig/main.py