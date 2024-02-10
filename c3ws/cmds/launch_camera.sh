#!/bin/bash

dt=`(uptime)`
echo -e "\n ${dt}"
echo -e "LAUNCHING OAK-D-LITE camera.launch.py WITH params/camera.yaml (mobilenet + RGBD)"
echo -e "ros2 launch depthai_ros_driver camera.launch.py camera_model:=OAK-D-LITE params_file:=/home/pi/wali_pi5/c3ws/params/camera.yaml\n"
ros2 launch depthai_ros_driver camera.launch.py camera_model:=OAK-D-LITE params_file:=/home/pi/wali_pi5/c3ws/params/camera.yaml
# ros2 launch depthai_ros_driver camera.launch.py camera_model:=OAK-D-LITE params_file:=/home/pi/wali_pi5/c3ws/params/oakd_lite.yaml

dt=`(uptime)`
echo -e "${dt}\n"
