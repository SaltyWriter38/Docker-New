once you have everything working and have followed HOW TO RUN ME:

build:

cd ros2_ws
colcon build --packages-select px4_msgs px4_offboard
source install/setup.bash

make sure px4 is running (should be already)
make sure the MicroXRCEAgent is running (should be already)
ros2 run px4_offboard offboard_test

--------------------------------------------------------------------------

these are the steps i followed to get here so we can reproduce / do something else in another script:

make a workspace:

cd ros2_ws/src
ros2 pkg create --build-type ament_python NAME-OF-YOUR-PACKAGE-GOES-HERE

add the code in YOUR-PACKAGE/YOUR-PACKAGE/whatever.py
and give it +x with chmod (needed?)

update package.xml in YOUR-PACKAGE to add dependencies from your script
i.e. add
<depend>rclpy</depend>
<depend>px4_msgs</depend>
inside the <package> tag

update setup.py in YOUR-PACKAGE
i had to:
import os
from glob import glob
and update 'entry_points' to point to main in my script
entry_points={
    'console_scripts': [
        'COMMAND-NAME = YOUR-PACKAGE.whatever:main'
    ],
},

build and compile
this took 20 mins the first time, hopefully should be faster if you only change the script and it doesnt have to rebuild px4
cd ~/ros2_ws
colcon build --packages-select px4_msgs YOUR-PACKAGE
source install/setup.bash

run:
make sure px4 is running (should be already)
make sure the MicroXRCEAgent is running (should be already)
ros2 run YOUR-PACKAGE COMMAND-NAME

todo: make a launch script?