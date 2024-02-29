#!/bin/bash


# export ROS_DISCOVERY_SERVER="10.0.0.219:11888;127.0.0.1:11888"
# export FASTRTPS_DEFAULT_PROFILES_FILE=/home/pi/wali_pi5/configs/super_client_configuration_file1.xml

# echo -e "\n*** Start FastDDS Discovery Server"
echo '*** fastdds discovery -i 1 -l 10.0.0.219 -p 11888 -l 127.0.0.1 -p 11888 &'
fastdds discovery -i 1 -l 10.0.0.219 -p 11888 -l 127.0.0.1 -p 11888 &

