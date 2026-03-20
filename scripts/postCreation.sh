#!/bin/bash
# We disable 'set -e' so we can see errors if they happen
# set -e 

echo ">>> STARTING POST-CREATION SETUP <<<"

sudo chown -R $USER:$USER /home/developer/workspace
git config --global --add safe.directory /home/developer/workspace

# 1. Run Helper Script
#if [ -f "/home/developer/scripts/helper.sh" ]; then
#    echo "Generating helper scripts..."
#    source /home/developer/scripts/helper.sh
#else
#    echo "WARNING: helper.sh not found"
#fi

cd /home/developer/workspace

# 2. Clone PX4
if [ ! -d "PX4-Autopilot" ]; then
    echo "Cloning PX4-Autopilot release 1.14..."
    git clone -b release/1.14 https://github.com/PX4/PX4-Autopilot.git --recursive
else
    echo "PX4-Autopilot already exists."
fi

# 3. INSTALL PYTHON DEPENDENCIES (Crucial Fix)
if [ -f "PX4-Autopilot/Tools/setup/requirements.txt" ]; then
    echo "Installing PX4 Python dependencies..."
    pip3 install --user -r PX4-Autopilot/Tools/setup/requirements.txt
else
    echo "WARNING: requirements.txt not found!"
fi

# 4. Clone ROS2 repos
mkdir -p ros2_ws/src
cd ros2_ws/src

if [ ! -d "px4_msgs" ]; then
    git clone -b release/1.14 https://github.com/PX4/px4_msgs.git
fi

if [ ! -d "px4_ros_com" ]; then
    git clone -b release/v1.14 https://github.com/PX4/px4_ros_com.git
fi

echo "Building PX4..."
bash /home/developer/scripts/build_px4.sh

echo "Building ROS2 Workspace..."
bash /home/developer/scripts/build_ros2.sh

echo ">>> SETUP AND BUILD COMPLETE! <<<"
