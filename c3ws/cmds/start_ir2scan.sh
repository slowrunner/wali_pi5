#!/bin/bash

export FASTRTPS_DEFAULT_PROFILES_FILE=/home/pi/wali_pi5/configs/fastdds-passive-unicast.xml

echo -e "\n **** STARTING ir2scan "
echo -e "executing: ros2 run wali ir2scan &"
ros2 run wali ir2scan &

