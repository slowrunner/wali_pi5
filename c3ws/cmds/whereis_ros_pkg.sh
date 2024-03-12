#!/bin/bash

# FILE:  whereis_ros_pkg.sh
# USAGE:  cmds/whereis_ros_pkg.sh [pkg name]

if [ "$#" -ne 1 ] ;
	then echo "Usage:  ./whereis_ros_pkg.sh depthai_ros_driver) "
	exit
fi
ros2 pkg prefix $1
