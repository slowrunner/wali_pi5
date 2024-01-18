#!/bin/bash

echo -e "\n*** PUB STOP /CMD_VEL"
echo -e 'ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"'
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.00, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"


