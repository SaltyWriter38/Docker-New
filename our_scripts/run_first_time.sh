cd ../ros2_ws
colcon build --packages-select px4_msgs gpig
source install/setup.bash
ros2 run gpig start