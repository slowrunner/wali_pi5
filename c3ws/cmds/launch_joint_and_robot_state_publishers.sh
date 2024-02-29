#!/bin/bash

echo -e "\n*** LAUNCHING ROBOT AND JOINT STATE PUBLISHERS with Create3-WaLI URDF"
echo -e "ros2 launch wali_description robot_description.launch.py &"
ros2 launch wali_description robot_description.launch.py &


