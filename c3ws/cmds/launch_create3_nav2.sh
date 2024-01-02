#!/bin/bash

echo -e "\n*** STARTING ROS2 Create3 NAVIGATION NAV2 and Localization w floorplan"
echo "*** Need initial pose?"
echo "*** ros2 launch create3_navigation localization.launch.py map:=floorplan.map.yaml &"
# ros2 launch create3_navigation localization.launch.py 'map:=/home/pi/wali_pi5/c3ws/maps/floorplan/floorplan.map.yaml'
# ros2 launch create3_navigation localization.launch.py 'map:=/home/pi/wali_pi5/c3ws/maps/xyzzy.map.yaml'
# ros2 launch create3_navigation localization.launch.py 

echo "*** ros2 launch create3_navigation nav2.launch.py "
ros2 launch create3_navigation nav2.launch.py
