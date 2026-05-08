#!/bin/bash

export GZ_SIM_RESOURCE_PATH="/home/developer/workspace/PX4-Autopilot/Tools/simulation/gz/models:${GZ_SIM_RESOURCE_PATH}"

# install mediapipe dependency for tom's code with compatible NumPy version
python3 -m pip install --user "numpy<2" >/dev/null 2>&1
if ! python3 -c "import mediapipe" >/dev/null 2>&1; then
    echo "INSTALLING MEDIAPIPE"
    python3 -m pip install --user mediapipe
fi

# if px4 or gazebo are running, kill them
pkill -f "bin/px4"
pkill -f "gz sim"
pkill -f "MicroXRCEAgent"

# get the right version of ros bridge
sudo apt remove -y \
  ros-humble-ros-gz-bridge \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-interfaces \
  ros-humble-ros-gz-image \
  ros-humble-ros-gz
sudo apt install -y ros-humble-ros-gzgarden

# we always want to run the MicroXRCE agent
MicroXRCEAgent udp4 -p 8888 &

# run the bridge for images
source /opt/ros/humble/setup.bash
ros2 run ros_gz_bridge parameter_bridge "/camera@sensor_msgs/msg/Image[gz.msgs.Image" &

WORLD_FILE="/home/developer/workspace/worlds/baylands.sdf"

gz sim -r -s "${WORLD_FILE}" &
GZ_PID=$!
gz sim -g &

echo "Waiting for Gazebo to start..."
for i in $(seq 1 30); do
    if gz topic -l 2>/dev/null | grep -q '/world/.*/clock'; then
        echo "Gazebo is ready."
        break
    fi
    sleep 1
done

# ---- Launch PX4, telling it to ATTACH to the existing drone defined in the SDF ----
cd /home/developer/workspace/PX4-Autopilot/build/px4_sitl_default/rootfs
PX4_GZ_MODEL_NAME=x500_depth_0 ../bin/px4


# ---- NEW: Manually spawn the drone at (20, 0, 0.2) before PX4 starts ----
DRONE_MODEL_SDF="/home/developer/workspace/PX4-Autopilot/Tools/simulation/gz/models/x500_depth/model.sdf"
DRONE_NAME="x500_depth_0"

echo "Spawning ${DRONE_NAME} at (-10, 0, 0.2)..."
gz service -s /world/baylands/create \
    --reqtype gz.msgs.EntityFactory \
    --reptype gz.msgs.Boolean \
    --timeout 5000 \
    --req "sdf_filename: \"${DRONE_MODEL_SDF}\", name: \"${DRONE_NAME}\", pose: {position: {x: -10, y: 0, z: 0.2}}"

sleep 2  # give Gazebo a moment to finish placing it

# launch px4 - tell it to ATTACH to the already-spawned drone (don't spawn a new one)
cd /home/developer/workspace/PX4-Autopilot/build/px4_sitl_default/rootfs
PX4_GZ_MODEL_NAME=${DRONE_NAME} ../bin/px4
