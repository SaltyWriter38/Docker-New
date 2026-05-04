#!/bin/bash

# install mediapipe dependency for tom's code with compatible NumPy version
# MediaPipe requires NumPy 1.x, not 2.x - see https://github.com/google/mediapipe/issues
python3 -m pip install --user "numpy<2" >/dev/null 2>&1
if ! python3 -c "import mediapipe" >/dev/null 2>&1; then
	echo "INSTALLING MEDIAPIPE"
	python3 -m pip install --user mediapipe
fi

#if px4 or gazebo are running, kill them
pkill -f "bin/px4" 
pkill -f "gz sim"

# get the right version of ros bridge 
sudo apt remove -y \
  ros-humble-ros-gz-bridge \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-interfaces \
  ros-humble-ros-gz-image \
  ros-humble-ros-gz
sudo apt install -y ros-humble-ros-gzgarden


#we always want to run the MicroXRCE agent
MicroXRCEAgent udp4 -p 8888 &

# run the bridge for images
source /opt/ros/humble/setup.bash
ros2 run ros_gz_bridge parameter_bridge /camera@sensor_msgs/msg/Image[gz.msgs.Image &


#launch gazebo with the right world file and drone model
WORLD_FILE="/home/developer/workspace/worlds/baylands.sdf"

#launch gazebo before px4 - px4 detects if gz is running when it starts
gz sim -r -s "${WORLD_FILE}" &
GZ_PID=$!
gz sim -g &

echo "Waiting for Gazebo to start..."
#wait 30s for gazebo to start up
for i in $(seq 1 30); do
	if gz topic -l 2>/dev/null | grep -q '/world/.*/clock'; then
		echo "Gazebo is ready."
		break
	fi
	sleep 1
done

#launch px4 with the gz_x500_depth model for the camera output
cd /home/developer/workspace/PX4-Autopilot/build/px4_sitl_default/rootfs
PX4_SIM_MODEL=gz_x500_depth ../bin/px4