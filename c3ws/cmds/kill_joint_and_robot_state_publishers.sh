#!/bin/bash

echo -e "\n*** KILL JOINT AND ROBOT STATE PUBLISHERS ***"
echo -e "killall joint_state_publisher"
echo -e "killall robot_state_publisher"
killall joint_state_publisher
killall robot_state_publisher
