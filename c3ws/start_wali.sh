#!/bin/bash

# This script is run when Docker container r2hdp starts
echo -e "START_WALI.SH EXECUTING"
# cd ~/wali_pi5/c3ws
source install/setup.bash
ros2 run wali wali_node 

# Use ./attach_to_docker_r2hdp.sh to connect to this terminal
# Use .term_to_r2hdp.sh to open a new terminal session 
#    in the ROS 2 Humble Desktop Plus container
