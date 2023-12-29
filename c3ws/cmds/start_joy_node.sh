#!/bin/bash

# REQ: apt-get install ros-<rosdistro>-teleop-twist-joy to install.


echo -e "\n*** Switching to ~/pi5desk/c3ws"
cd ~/wali_pi5/c3ws

echo -e "\n*** Sourcing install/setup.bash"
. ~/wali_pi5/c3ws//install/setup.bash

echo -e "\n*** Start F710 game controller node"
echo '*** ros2 launch teleop_twist_joy teleop-launch.py joy_config:="F710" '
ros2 launch teleop_twist_joy teleop-launch.py joy_config:="F710" &

