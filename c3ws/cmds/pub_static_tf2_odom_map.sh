#!/bin/bash

# Publish static transform from odom to map
# REF: https://docs.ros.org/en/eloquent/Tutorials/tf2.html

echo -e "ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom map"
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom map

