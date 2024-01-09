#!/bin/bash

# This script is run when Docker container r2hdp starts
echo -e "START_WALI.SH EXECUTING"
cd /home/pi/wali_pi5/c3ws
source install/setup.bash

echo -e "\n*** Start F710 game controller node"
echo '*** ros2 launch teleop_twist_joy teleop-launch.py joy_config:="F710" '
ros2 launch teleop_twist_joy teleop-launch.py joy_config:="F710" &

echo -e "\n*** Start odometer node"
echo '*** ros2 run wali odometer & '
ros2 run wali odometer &

echo -e "\n*** Start wali.say_node"
echo '*** ros2 run wali say_node &'
ros2 run wali say_node &

# MUST START IN FOREGROUND or docker will exit!
ros2 run wali wali_node

# Use ./attach_to_docker_r2hdp.sh to connect to this terminal
# Use .term_to_r2hdp.sh to open a new terminal session 
#    in the ROS 2 Humble Desktop Plus container
