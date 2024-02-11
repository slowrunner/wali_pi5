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
