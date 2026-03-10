#!/bin/bash

# Path to your custom world file (kept outside PX4 so it survives rebuilds)
WORLD_FILE="/home/developer/workspace/worlds/baylands.sdf"
#WORLD_FILE="/home/developer/workspace/worlds/small_city.sdf"

# Launch Gazebo with the custom world before starting PX4.
# PX4 will detect gz-sim is already running and connect to it instead of
# launching its own world.
gz sim -r -s "${WORLD_FILE}" &
GZ_PID=$!

# Launch the Gazebo GUI (visible in VNC)
gz sim -g &

# Wait for Gazebo to come up (its /world/<name>/clock topic to appear)
echo "INFO  Waiting for Gazebo to start..."
for i in $(seq 1 30); do
	if gz topic -l 2>/dev/null | grep -q '/world/.*/clock'; then
		echo "INFO  Gazebo is ready."
		break
	fi
	sleep 1
done

cd /home/developer/workspace/PX4-Autopilot/build/px4_sitl_default/rootfs
PX4_SIM_MODEL=gz_x500 ../bin/px4
