#!/bin/bash

echo -e "\n** DOCK **"
if [ $ROS_DISTRO == "galactic" ]
  then
    echo -e '** ros2 action send_goal /dock irobot_create_msgs/action/DockServo "{}"'
    ros2 action send_goal /dock irobot_create_msgs/action/DockServo "{}"
  else
    echo -e '** ros2 action send_goal /dock irobot_create_msgs/action/Dock "{}"'
    ros2 action send_goal /dock irobot_create_msgs/action/Dock "{}"
fi
/home/pi/wali_pi5/utils/logMaintenance.py 'Manual Docking: success (assumed)'

/home/pi/wali_pi5/c3ws/cmds/reset_pose.sh


