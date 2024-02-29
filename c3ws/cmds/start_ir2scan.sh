#!/bin/bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/pi/wali_pi5/configs/super_client_configuration_file1.xml
export ROS_DISCOVERY_SERVER="127.0.0.1:11888"

echo -e "\n **** STARTING ir2scan "
echo -e "executing: ros2 run wali ir2scan &"
ros2 run wali ir2scan1 &

