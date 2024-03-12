#!/bin/bash


export FASTRTPS_DEFAULT_PROFILES_FILE=/home/pi/wali_pi5/configs/fastdds-passive-unicast.xml
# ros2 daemon stop

echo -e "\n*** Switching to ~/pi5desk/c3ws"
cd ~/wali_pi5/c3ws

echo -e "\n*** Sourcing install/setup.bash"
. ~/wali_pi5/c3ws//install/setup.bash

echo -e "\n*** Start odometer node"
echo '*** ros2 run wali odometer & '
ros2 run wali odometer &

