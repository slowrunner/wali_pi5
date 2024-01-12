#!/bin/bash

# FILE: cmds/start_coverage_node

# Starts the Create3 Coverage Node

# In a second (ROS) terminal  send /coverage goal with cmds/send_coverage.sh
# (On Pi5 with ROS 2 Humble running in Docker container, 
#    the user must first attach the terminal to the ROS container,
#    with ./term_to_r3hdp.sh in my case)

ros2 run create3_coverage create3_coverage


