#!/bin/bash

cd ~/wali_pi5/c3ws

echo -e "Launching map_server (with map.yaml which points to floorplan.map.pgm)"
echo -e "ros2 launch create3_navigation map_server.launch.py"
ros2 launch create3_navigation map_server.launch.py
