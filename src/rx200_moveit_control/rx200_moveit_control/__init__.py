# ## for assignment_1
# cd assignment_1/
# colcon build
# source install/setup.bash
# ros2 run rx200_moveit_control rx200_moveit_client



# cd interbotix_ws/

# # connect to robot (rviz pops up)
# ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=rx200

# # below disables motors (puts it in neutral)
# ros2 service call /rx200/torque_enable interbotix_xs_msgs/srv/TorqueEnable "{cmd_type: 'group', name: 'all', enable: false}"

# ros2 service call /rx200/torque_enable interbotix_xs_msgs/srv/TorqueEnable "{cmd_type: 'group', name: 'all', enable: true}"
