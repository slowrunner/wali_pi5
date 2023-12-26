#!/bin/bash


echo -e '\n*** Appending: export ROS_LOG_DIR="/home/pi/wali_pi5/c3ws/roslogs"' to ~/.bashrc
echo -e '\nexport ROS_LOG_DIR="/home/pi/wali_pi5/c3ws/roslogs"' >> ~/.bashrc

echo -e '\n*** Appending: export ROS_DISTRO="humble"' to ~/.bashrc
echo 'export ROS_DISTRO="humble"' >> ~/.bashrc

echo -e '\n** Appending source ss.sh if exists to ~/.bashrc'
echo -e '\n' >> ~/.bashrc
echo "if [ -f /home/pi/wali_pi5/c3ws/ss.sh ]; then " >> ~/.bashrc
echo "    source /home/pi/wali_pi5/c3ws/ss.sh " >> ~/.bashrc
echo "fi" >> ~/.bashrc

echo -e '\n# If ssh closes immediately, use "ssh -t pi@x.x.x.x /bin/sh" and fix ~/.bashrc' >> ~/.bashrc



echo -e '\n*** tail ~/.bashrc:'
tail ~/.bashrc
echo -e '\n'
