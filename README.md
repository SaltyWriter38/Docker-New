Hello lovely team. Please find instructions below on how to run the container for development.
First you need to clone the repo onto your machine - at the time of writing the container doesn't work on the clone remote repository. This turns out to be a good thing as the first time build time on Windows/Mac is 20+ minutes - Linux is much faster. 

-------------

NEW ACTUAL README:
1. Follow HOW TO RUN ME.md
2. Follow HOW TO RUN THE TEST.md

--------------

Steps for Windows (MAC is mostly similar but needs further testing w/Luke help):

1. Clone the repo locally and open it in vscode - ensure you have the devcontainers extension downloaded. 
2. Change to the testing-VNC-hybrid branch (at the time of writing). 

3. Ctrl + Shift + P and type in "dev containers: reopen in container", selecting the rebuild and reopen in container/re-open in container option. 

4. Monitor the build log to track progress on container build. 

5. Once complete run the commands found in the Instructions to get VNC running before running any other scripts to get PX4 running or building the ROS envirnoment. 

6. Develop to hearts content :)

Steps for Linux (tested on Ubuntu - imagine similar for non-debian distros):

1. Install docker locally and add your user to the docker group.
	- sudo usermod -aG docker $USER
	- newgrp docker 
	- docker ps (if there is any form)

2. Run in a local terminal (not vs code):
	- xhost +local:docker
This allows for the docker to draw on the machine screen/monitor but also enables gpu passthrough. 

Repeat the steps stated above and voila it should be all working.
