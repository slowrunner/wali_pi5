#!/bin/bash

if [ $ROS_DISTRO == "galactic" ]; then
    source /opt/ros/galactic/setup.bash
  else
    source /opt/ros/humble/setup.bash
fi

source ~/wali_desk/c3ws/install/setup.bash

