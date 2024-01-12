#!/bin/bash

# FILE:  cmds/send_coverage_goal.sh

# USAGE:  cmds/send_coverage_goal.sh  exploreDurationSeconds maxRuntimeSeconds
#         (after exploreDuration expires if there is runtime remaining the action will look for and dock if possible)
if [ "$#" -ne 2 ] ;
  then echo -e "\n*** USAGE: cmds/send_coverage_goal.sh exploreDurationSeconds maxRuntimeSeconds\n"
       exit
fi

# Convert parms to string type
fExplore=$1
fMaxRun=$2

echo -e "Sending /coverage goal {explore_duration: $fExplore  max_runtime: $fMaxRun }"
ros2 action send_goal /coverage create3_examples_msgs/action/Coverage "{explore_duration:{sec: $1, nanosec: 0}, max_runtime:{sec: $2,nanosec: 0}}"
