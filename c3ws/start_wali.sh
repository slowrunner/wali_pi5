#!/bin/bash

# This script is run when Docker container r2hdp starts
echo -e "START_WALI.SH EXECUTING"
cd /home/pi/wali_pi5/c3ws
source install/setup.bash

# echo -e "\n*** Start FastDDS Discovery Server 0"
echo '*** fastdds discovery -i 0 -l 192.168.186.3 -p 11811 -l 10.0.0.219 -p 11811 -l 127.0.0.1 -p 11811 &'
fastdds discovery -i 0 -l 192.168.186.3 -p 11811 -l 10.0.0.219 -p 11811 -l 127.0.0.1 -p 11811 &

# echo -e "\n*** Start FastDDS Discovery Server 1"
# echo '*** fastdds discovery -i 1 -l 10.0.0.219 -p 11888 -l 127.0.0.1 -p 11888 &'
# fastdds discovery -i 1 -l 10.0.0.219 -p 11888 -l 127.0.0.1 -p 11888 &

# Use Discovery Server 0 for all these now
export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/pi/wali_pi5/configs/super_client_configuration_file0.xml


# sleep to allow fastdds server startup?
sleep 5

echo -e "\n*** Start F710 game controller node"
echo '*** ros2 launch teleop_twist_joy teleop-launch.py joy_config:="F710" '
ros2 launch teleop_twist_joy teleop-launch.py joy_config:="F710" &

echo -e "\n*** Start odometer node"
echo '*** ros2 run wali odometer & '
ros2 run wali odometer &

echo -e "\n*** Start wali.say_node"
echo '*** ros2 run wali say_node &'
ros2 run wali say_node &

# echo -e "\n **** STARTING ir2scan "
# echo -e "executing: ros2 run wali ir2scan &"
# ros2 run wali ir2scan &

# MUST START IN FOREGROUND or docker will exit!
ros2 run wali wali_node

# Use ./attach_to_docker_r2hdp.sh to connect to this terminal
# Use .term_to_r2hdp.sh to open a new terminal session 
#    in the ROS 2 Humble Desktop Plus container
