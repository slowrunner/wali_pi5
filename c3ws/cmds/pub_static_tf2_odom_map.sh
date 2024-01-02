#!/bin/bash

# Publish static transform from odom to map

echo -e "ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom map"
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom map

