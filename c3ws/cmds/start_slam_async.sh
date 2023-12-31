#!/bin/bash


echo -e "\n*** STARTING ROS2 SLAM-TOOLBOX with params/slam.Create3.yaml"
echo "*** Drive Wali around room, generating /map topics"
echo "*** (Async: Process scans as able to, online))"
echo "*** ASSUMES IR2SCAN NODE IS RUNNING"
echo "*** ros2 launch slam_toolbox online_async_launch.py 'slam_params_file:=/home/pi/wali_pi5/c3ws/params/map.create3.yaml"
ros2 launch slam_toolbox online_async_launch.py 'slam_params_file:=/home/pi/wali_pi5/c3ws/params/map.create3.yaml'

