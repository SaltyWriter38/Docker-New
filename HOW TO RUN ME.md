Follow these instructions to fully set up the container. 

1. Open Docker, skip sign-in (unless you want to sign in). 

2. Open VS Code. 

3. Install the 'Dev Containers' extension in VS Code. 

4. Clone the repository TO YOUR C: DRIVE (OR ANOTHER LOCAL DRIVE). Nowhere else. 

5. Navigate into the folder containing the Docker Container. 

+ CHECKOUT TO THE MOST RECENT BRANCH

6. Control + Shift + P > 'Dev Containers: Open Folder in Container'. 

7. Select container type (VNC always works, but is slow). 

8. Wait for the build (20-30 minutes). 

9. Wait for all the submodules to install automatically (5 minutes). 

10. Run this command: /home/developer/workspace/scripts/build_px4.sh

11. Wait for the message 'ready for takeoff!' or something. 

12. In that terminal, type 'commander takeoff'. 

13. In another terminal, type: MicroXRCEAgent udp4 -p 8888 (this makes a bridge between ros2 and PX4).

14. Play with the drone (good luck). 

--- 

PS: If you get locked out of the virtual machine, the password seems to be 'developer'. 