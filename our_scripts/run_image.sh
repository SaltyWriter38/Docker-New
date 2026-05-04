cd ../ros2_ws
colcon build --packages-select gpig
source install/setup.bash
ros2 run gpig object_detection --ros-args -p image_topic:=/camera -p show_debug_windows:=true -p model_name:=efficientdet_lite2.tflite