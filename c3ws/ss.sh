#!/bin/bash

if [ -f /opt/ros/galactic/setup.bash ]; then
    source /opt/ros/galactic/setup.bash
fi

if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

if [ -f /opt/ros/humble/local_setup.bash ]; then
  if [ -f ~/wali_pi5/c3ws/install/setup.bash ]; then
   source ~/wali_pi5/c3ws/install/setup.bash
  fi
fi
