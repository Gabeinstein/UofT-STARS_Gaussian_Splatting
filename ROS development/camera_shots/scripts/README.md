# Use of data_acquisition.py

This is an automation code to take camera shots or display in real time the topic panda/camera1/image_raw.

### Format

rosrun camera_shots data_acquisition.py <node_name> <mode(0-1)> <theta_angle(0-10)> <number_of_shots> <saving_path>

"Display Setup"

- To display with python the images from image_raw you should run in console the following.

Example: rosrun camera_shots data_acquisition.py display_node 1 0 0 0

"Take camera shots to a specific directory"

- To take camera shots from image_raw you can set up the following parameters:

  - Theta angle with range (0 - 10). 0 level is on the floor and 10 level is at the top of the robot.

  - Number of shots: choose how many shots you want to take

  - saving_path: Give the path to the folder where you want to store the jpg images. -> /home/path/to/folder/

Example: rosrun camera_shots data_acquisition.py screenshots_node 0 5 30 /home/path/to/saving/directory/
