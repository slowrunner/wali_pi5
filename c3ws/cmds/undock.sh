#!/bin/bash

echo -e "\n** UNDOCK (Galactic same as Humble) **"
echo -e '** ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}"'
ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}"
/home/pi/wali_pi5/utils/logMaintenance.py "Manual Undocking: success (assumed)"
