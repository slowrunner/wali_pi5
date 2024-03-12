#!/bin/bash

export FASTRTPS_DEFAULT_PROFILES_FILE=~/wali_pi5/configs/fastdds-passive-unicast.xml

echo -e "\n*** Start wali.say_node"
echo '*** ros2 run wali say_node &'
ros2 run wali say_node &

