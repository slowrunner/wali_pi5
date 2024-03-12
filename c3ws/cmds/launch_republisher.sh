#!/bin/bash


# Launch the republisher with the active profile
export FASTRTPS_DEFAULT_PROFILES_FILE=~/wali_pi5/configs/fastdds-active-unicast.xml
ros2 daemon stop
ros2 launch create3_republisher create3_republisher_launch.py  robot_ns:=/wali republisher_ns:=/ &

export FASTRTPS_DEFAULT_PROFILES_FILE=~/wali_pi5/configs/fastdds-passive-unicast.xml
ros2 daemon stop

