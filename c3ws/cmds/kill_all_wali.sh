#!/bin/bash

# FILE: kill_all_wali.sh

# Stops all Wali ROS 2 nodes including wali_node
# by sending "KeyboardInterrupt" which is SIGINT

echo -e "\n**** KILL *ALL* WALI ROS 2 NODES (including wali_node)"
echo -e "Killing ir2scan"
pkill --signal SIGINT ir2scan 2> /dev/null
echo -e "Killing joy_node"
pkill --signal SIGINT joy_node 2> /dev/null
echo -e "Killing teleop_node"
pkill --signal SIGINT teleop_node 2> /dev/null
echo -e "Killing odometer"
pkill --signal SIGINT odometer 2> /dev/null
echo -e "KILLING WALI_NODE"
pkill -e --signal SIGINT wali_node
echo -e "**** WAITING 5s FOR PROCESS TO FINISH ****"
sleep 5
echo -e "\n**** DONE"
