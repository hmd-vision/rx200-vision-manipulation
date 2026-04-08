Project description:
Pick and Place code for a ReactorX-200. 

This repo contains:
- a small Tkinter GUI (gui_publisher) that publishes coordinates. The GUI does IK/reach checks before publishing to the rx200_moveit_action_client.
- a MoveIt action client (rx200_moveit_action_client) that receives pick/place coordinates and executes them using MoveIt

Requirements:

- ROS 2 installed.
- MoveIt2 installed and the Interbotix MoveIt packages available in your system.
- Project files in ~/assignment_1

To Run:

With Launch File:
-- Run in cmd -- 
cd ~/assignment_1/
rm -rf build/ install/ log/
colcon build
source install/setup.bash
ros2 launch rx200_moveit_control launch_all.launch.py robot_type:=fake default_gripper_state:=false

Note: The fake robot uses robot_type:=fake, which simulates the robotic arm.

Without Launch File:
-- Run in cmd terminal 1 -- 
cd interbotix_ws/
ros2 launch interbotix_xsarm_moveit xsarm_moveit.launch.py robot_model:=rx200 hardware_type:=actual

Note: The fake robot uses hardware_type:=fake, which simulates the robotic arm.

-- Run in cmd terminal 2 -- 
cd assignment_1/
colcon build
source install/setup.bash
ros2 run rx200_moveit_control rx200_moveit_client

-- Run in cmd terminal 3 -- 
cd assignment_1/
colcon build
source install/setup.bash
ros2 run rx200_moveit_control keyboard_gui


Notes and limitations:
- No RViz trajectory visualization; the current client focuses on execution.
- No Gazebo simulation included in this repo.
- Linear Cartesian motions - this implementation uses MoveIt planning by default and does not explicitly enforce Cartesian straight-line end-effector trajectories.

Team 4 Team Members:
Haleemah Hmedan
Krystal Davis
subhashini reddy kunduru