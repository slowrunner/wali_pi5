#!/bin/bash

# export ROS_DISCOVERY_SERVER="127.0.0.1:11888"
# export FASTRTPS_DEFAULT_PROFILES_FILE=/home/pi/wali_pi5/configs/super_client_configuration_file1.xml

export FASTRTPS_DEFAULT_PROFILES_FILE=/home/pi/wali_pi5/configs/fastdds-passive-unicast.xml

dt=`(uptime)`
echo -e "\n ${dt}"
echo -e "LAUNCHING OAK-D-LITE camera.launch.py WITH params/camera.yaml (mobilenet + RGBD)"
echo -e "ros2 launch depthai_ros_driver camera.launch.py camera_model:=OAK-D-LITE params_file:=/home/pi/wali_pi5/c3ws/params/camera.yaml\n"
ros2 launch depthai_ros_driver camera.launch.py camera_model:=OAK-D-LITE params_file:=/home/pi/wali_pi5/c3ws/params/camera.yaml
# ros2 launch depthai_ros_driver camera.launch.py camera_model:=OAK-D-LITE params_file:=/home/pi/wali_pi5/c3ws/params/camera.yaml rsp_use_composition:=false
# ros2 launch depthai_ros_driver camera.launch.py camera_model:=OAK-D-LITE params_file:=/home/pi/wali_pi5/c3ws/params/oakd_lite.yaml

dt=`(uptime)`
echo -e "${dt}\n"
