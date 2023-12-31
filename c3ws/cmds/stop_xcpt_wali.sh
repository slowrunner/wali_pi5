#!/bin/bash

# FILE: stop_xcpt_wali.sh

# Stops all Wali ROS 2 nodes EXCEPT wali_node
# by sending "KeyboardInterrupt" which is SIGINT

echo -e "\n**** KILL WALI ROS 2 NODES (EXCEPT wali_node)"
echo -e "Killing ir2scan"
pkill --signal SIGINT ir2scan 2> /dev/null
echo -e "Killing joy_node"
pkill --signal SIGINT joy_node 2> /dev/null
echo -e "Killing teleop_node"
pkill --signal SIGINT teleop_node 2> /dev/null
echo -e "Killing odometer"
pkill --signal SIGINT odometer 2> /dev/null

echo -e "**** WAITING 5s FOR PROCESS TO FINISH ****"
sleep 5
echo -e "\n**** DONE"
