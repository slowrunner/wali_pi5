#!/bin/bash

echo -e "\n *** Using Discovery Server 0 and 1 ***"
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/pi/wali_pi5/configs/super_client_configuration_file01.xml
export ROS_DISCOVERY_SERVER="127.0.0.1:11811;127.0.0.1:11888"

echo -e "\n **** STARTING ir2scan1 "
# echo -e "executing: ros2 run wali ir2scan1 --ros-args --remap __node:=ir2scan1 &"
echo -e "executing: ros2 run wali ir2scan1 &"
ros2 run wali ir2scan1 &

