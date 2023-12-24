#!/bin/bash


echo -e '\n*** Appending: export ROS_LOG_DIR="/home/pi/wali_pi5/c3ws/roslogs" to ~/.bashrc' 
echo 'export ROS_LOG_DIR="/home/pi/wali_pi5/c3ws/roslogs"' >> ~/.bashrc
echo -e '\n*** tail ~/.bashrc:'
tail ~/.bashrc
echo -e '\n'
