#!/bin/bash

echo -e "Config for Discovery Server 0"
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/pi/wali_pi5/configs/super_client_configuration_file0.xml
export ROS_DISCOVERY_SERVER="127.0.0.1:11811"

echo -e `date +"%A %D %H:%M:%S"`
echo -e 'ros2 run wali c3interface --ros-args --remap __node:=c3interface &'

ros2 run wali c3interface &

echo -e `date +"%A %D %H:%M:%S"`

