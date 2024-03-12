#!/bin/bash

# export FASTRTPS_DEFAULT_PROFILES_FILE=/home/pi/wali_pi5/configs/super_client_configuration_file0.xml
# export ROS_DISCOVERY_SERVER="127.0.0.1:11811"

export FASTRTPS_DEFAULT_PROFILES_FILE=~/wali_pi5/configs/fastdds-passive-unicast.xml

echo -e `date +"%A %D %H:%M:%S"`

ros2 launch depthai_ros_driver rtabmap.launch.py params_file:=/home/pi/wali_pi5/c3ws/params/no_imu.yaml

echo -e `date +"%A %D %H:%M:%S"`

