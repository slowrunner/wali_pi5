#!/bin/bash

# FILE: apt_search_ROS_Humble_pkgs.sh

echo -e "\n*** USAGE: cmds/apt_search_ROS_Humble_pkgs.sh [pattern] ***\n"
sleep 5

if [ "$#" -ne 1 ] 
     then apt-cache search --names-only 'ros-humble-*' 
     else apt-cache search --names-only 'ros-humble-*' | grep $1
fi

