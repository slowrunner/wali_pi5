#!/bin/bash

# On Pi5:
# export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
# export FASTRTPS_DEFAULT_PROFILES_FILE=/home/pi/wali_pi5/configs/super_client_configuration_file0.xml

# On Desktop (to connect only to server 0)
# export ROS_DISCOVERY_SERVER="10.0.0.219:11811"
# export FASTRTPS_DEFAULT_PROFILES_FILE=/home/pi/wali_pi5/configs/super_client_configuration_file0Desk.xml

# On Create3 App Config Enable FastDDS Server: 192.168.186.3:11811


# echo -e "\n*** Start FastDDS Discovery Server 0"
echo '*** fastdds discovery -i 0 -l 10.0.0.219 -p 11811 -l 127.0.0.1 -p 11811 -l 192.168.186.3 -p 11811 &'
fastdds discovery -i 0 -l 10.0.0.219 -p 11811 -l 127.0.0.1 -p 11811 -l 192.168.186.3 -p 11811 &

