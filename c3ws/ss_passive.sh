#!/bin/bash

# if [ -f /opt/ros/galactic/setup.bash ]; then
#     source /opt/ros/galactic/setup.bash
# fi

if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

if [ -f /opt/ros/humble/local_setup.bash ]; then
  if [ -f ~/wali_pi5/c3ws/install/setup.bash ]; then
    source ~/wali_pi5/c3ws/install/setup.bash
    echo -e "sourced c3ws install setup.bash"
  fi

  if [ -f ~/wali_pi5/dai_ws/install/setup.bash ]; then
    source ~/wali_pi5/dai_ws/install/setup.bash
    echo -e "sourced dai_ws install setup.bash"
  fi
fi

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# export FASTRTPS_DEFAULT_PROFILES_FILE=/home/pi/wali_pi5/configs/super_client_configuration_file.xml
# export ROS_DISCOVERY_SERVER=127.0.0.1:11811

# Set up for republisher
export FASTRTPS_DEFAULT_PROFILES_FILE=~/wali_pi5/configs/fastdds-passive-unicast.xml
ros2 daemon stop

ros2 topic list
