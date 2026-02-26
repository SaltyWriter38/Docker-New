#!/bin/bash
source /opt/ros/humble/setup.bash
cd /home/developer/workspace/ros2_ws
colcon build --symlink-install
source install/setup.bash
