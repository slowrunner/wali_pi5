#!/bin/bash


echo -e "\n*** STARTING ROS2 SLAM-TOOLBOX "
echo "*** Drive Wali around room, generating /map topics"
echo "*** (Async: Process scans as able to, online))"
echo "*** ros2 launch slam_toolbox online_async_launch.py 'slam_params_file:=./my_mapper_params_online_async.yaml"
# ros2 launch slam_toolbox online_sync_launch.py 'slam_params_file:=/home/pi/wali_pi5/c3ws/params/my_mapper_params_online_async.yaml'
ros2 launch slam_toolbox online_sync_launch.py 'slam_params_file:=/home/pi/wali_pi5/c3ws/params/mapper_params_online_async.yaml'

