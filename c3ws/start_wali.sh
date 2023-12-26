#!/bin/bash

echo -e "START_WALI.SH EXECUTING"
# cd ~/wali_pi5/c3ws
source install/setup.bash
ros2 run wali wali_node
