# Rover Status Node for ROB499 Class Project

This repository contains a set of nodes designed to general & specific monitoring of ROS2 systems on the DAM Robotics Rover teams' robot. It was to fulfill the final assignment of ROB499 by Wyatt Boer and Osian Leahy.

This project works with ROS2 humble, and probably works with jazzy

# Functionality of Rover Status Node:

Dynamically determine what nodes are available and track if any have crashed/errored out. It is able to listen in to any node, and see what the node is publishing/subscribing to, as well as associated services. There is a service such that you can introspect a node’s logs from the UI.  
Additionally some handling of common motor driver data for the MoveIt stack, see motor status (error states, current, position, velocity, etc).  
UI built into tables in the terminal relatively neatly for easier viewing, as opposed to the never ending scrolling of an unformatted terminal.  



Grading Instructions:
1. Clone the code from this repository and checkout the appropriate branch. A branch titled "midterm-checkin"has been created for the initial assignment. The final assignment is the main branch.
2. insure that the "rich" python package is installed. it is available from apt with the following:
> sudo apt install python3-rich

3. Download the rosbag for testing MoveItLogger Functionality https://drive.google.com/file/d/1zJsqvi0TLlC8YZv380w0_dTCiOndK8iC/view?usp=sharing

4. Build the package in a ros2 workspace.

5. To test the functionality. Launch the following in ros:

> ros2 launch rob499_rover_status_ui DEMO_rover_ui.launch.py

In another terminal to run the actual UI run:  

>ros2 run rob499_rover_status_ui integrator --ros-args --disable-stdout-logs  

This will show tables containing node statuses, node communications, and simplified node log info, MoveItLogger info, and drivetrain info.

6. To confirm that the node statuses are being displayed properly, start any other ros2 node via ros run, and kill/restart it to confirm that it marks the node as alive or dead correctly.

7. To view and confirm that the node communications and log infoare being identified correctly, run the following parameter change to select a node to view (oscope and limiter nodes are being run as part of the Demo launch file):
> ros2 param set /integrator node_select "<NAME OF NODE>"

8. Switch this parameter between multiple nodes to confirm functionality

Note: When viewing node details, if the subscribers/publishers/services change after the node selection service call is ran, the details will not update until the service call for the node is ran again

Note: Node logs info is rate limited to 1hz by default (modifiable via a service call), but any log messages stronger than info level are passe through.

Note: By default due to settings in the launch file you cannot introspect the logs of the nodes from this package.
