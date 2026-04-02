#!/bin/bash

#if px4 or gazebo are running, kill them
pkill -f "bin/px4" 
pkill -f "gz sim"

#we always want to run the MicroXRCE agent
MicroXRCEAgent udp4 -p 8888 &

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