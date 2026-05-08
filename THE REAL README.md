Follow these meticulous instructions to actually run this shit

1. Run Docker Desktop

2. Install Dev Containers VS Code extension

3. Git clone the repo into your C drive, and cd into it from VS Code

4. Checkout to the correct branch (main for stable version, test-ghcr-image for development)

5. CMD->SHIFT->P -> Dev Containers: Open Folder in Container

6. Wait for the Docker container to build properly, until you get the message 'ready for takeoff'
    - then CTRL-C this terminal to get it to run postCreate.sh and run vnc

7. To run (all from /our_scripts/):
    - ./custom_run_sim.sh (wait for 'ready for takeoff')
    - then, ./run_image.sh
    - then, ./run_first_time.sh the first time you run the simulation, and ./run_second_time.sh thereafter
    - if you inevitably get one of these issues:
        - 'permission denied' - chmod +x filename.sh
        - 'bad interpreter' or 'no such file as ros2_ws' - sed -i -e 's/\r$//' filename.sh

8. Hope and pray that everything works the way you expect it to.

9. If Tom's code crashes, just rerun everything from step 7. again

10. If the drone doesn't take off, just rerun everything from step 7. again