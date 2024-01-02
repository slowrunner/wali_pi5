#!/bin/bash


# call map_server /load_map service
echo -e "Calling map_server /load_map service with map.yaml"
ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap "{map_url: /home/pi/wali_pi5/c3ws/install/create3_navigation/share/create3_navigation/maps/floorplan.map.yaml}"

