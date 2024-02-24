#!/bin/bash

export FASTRTPS_DEFAULT_PROFILES_FILE=/home/pi/wali_pi5/configs/super_client_configuration_file0.xml
export ROS_DISCOVERY_SERVER="127.0.0.1:11811"

echo -e `date +"%A %D %H:%M:%S"`

ros2 run wali c3interface &

echo -e `date +"%A %D %H:%M:%S"`

