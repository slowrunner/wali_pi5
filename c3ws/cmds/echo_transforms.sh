#!/bin/bash

# FILE: cmds/echo_transforms.sh

# NOTE: ros2 topic echo /tf  WILL KILL THE CREATE3 APPLICATION!!!
#       use instead:
echo -e "\n**** PRESS CNTRL-C after each command starts listing the respective transform"
echo -e "\nros2 run tf2_ros tf2_echo odom base_link"
ros2 run tf2_ros tf2_echo odom base_link
echo -e "\nros2 run tf2_ros tf2_echo odom base_footprint"
ros2 run tf2_ros tf2_echo odom base_footprint
