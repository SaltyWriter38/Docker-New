# Create helpful scripts directory
mkdir -p /home/developer/scripts

# Build PX4 firmware
cat > /home/developer/scripts/build_px4.sh << 'EOF'
#!/bin/bash
cd /home/developer/workspace/PX4-Autopilot
make clean
make px4_sitl gz_x500
EOF
chmod +x /home/developer/scripts/build_px4.sh

# Run PX4 SITL with Gazebo
cat > /home/developer/scripts/run_simulation.sh << 'EOF'
#!/bin/bash
cd /home/developer/workspace/PX4-Autopilot
make px4_sitl gz_x500
EOF
chmod +x /home/developer/scripts/run_simulation.sh

# Start the DDS bridge
cat > /home/developer/scripts/run_dds_agent.sh << 'EOF'
#!/bin/bash
MicroXRCEAgent udp4 -p 8888
EOF
chmod +x /home/developer/scripts/run_dds_agent.sh

# Build ROS2 workspace
cat > /home/developer/scripts/build_ros2.sh << 'EOF'
#!/bin/bash
source /opt/ros/humble/setup.bash
cd /home/developer/workspace/ros2_ws
colcon build --symlink-install
source install/setup.bash
EOF
chmod +x /home/developer/scripts/build_ros2.sh