cd ../ros2_ws
colcon build --packages-select gpig
source install/setup.bash
ros2 run gpig start