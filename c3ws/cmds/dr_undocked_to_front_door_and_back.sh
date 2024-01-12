#!/bin/bash

cd /home/pi/wali_pi5/c3ws

cmds/say.sh "I'm headed to the front door now"

echo -e "\n*** DEAD-RECKONING TO THE FRONT DOOR FROM UNDOCKED AND BACK***"

/home/pi/wali_pi5/utils/logMaintenance.py "Dead-Reckoning To The Front Door From Undocked (and back)"

echo -e "\n*** GOTO x: -0.670, y: 0.670  Q(45 deg about Z) "
ros2 action send_goal /navigate_to_position irobot_create_msgs/action/NavigateToPosition "{achieve_goal_heading: true, max_translation_speed: 0.1, max_rotation_speed: 0.5, goal_pose:{pose:{position:{x: -0.670, y: 0.670,z: 0.0}, orientation:{x: 0,y: 0, z: 0.3826834, w: 0.9238795}}}}"

cmds/say.sh "It feels good to be on the move"

echo -e "\n*** DRIVE TO CROSS PATHS POINT ***"

echo -e "\n*** GOTO x: 0.225, y: 1.830  Q(-45 deg about Z) "
ros2 action send_goal /navigate_to_position irobot_create_msgs/action/NavigateToPosition "{achieve_goal_heading: true, max_translation_speed: 0.1, max_rotation_speed: 0.5, goal_pose:{pose:{position:{x: 0.225, y: 1.830, z: 0.0}, orientation:{x: 0,y: 0, z: -0.3826834, w: 0.9238795}}}}"

echo -e "\n*** DRIVE TOWARD MASTER FOYER ***"

cmds/say.sh "On toward the master foyer"

echo -e "\n*** GOTO x: 0.910, y: 4.528  Q(0 deg about Z) "
ros2 action send_goal /navigate_to_position irobot_create_msgs/action/NavigateToPosition "{achieve_goal_heading: true, max_translation_speed: 0.1, max_rotation_speed: 0.5, goal_pose:{pose:{position:{x: 0.455, y: 4.778, z: 0.0}, orientation:{x: 0,y: 0, z: 0, w: 1.0}}}}"

echo -e "\n*** DRIVE TO FRONT DOOR ***"
cmds/say.sh "Now to check that the front door is locked"


echo -e "\n*** GOTO x: 4.095, y: 4.778  Q(0 deg about Z) "
ros2 action send_goal /navigate_to_position irobot_create_msgs/action/NavigateToPosition "{achieve_goal_heading: true, max_translation_speed: 0.1, max_rotation_speed: 0.5, goal_pose:{pose:{position:{x: 4.095, y: 4.778, z: 0.0}, orientation:{x: 0,y: 0, z: 0, w: 1}}}}"

/home/pi/wali_pi5/utils/logMaintenance.py "Arrived at front door"

cmds/say.sh "Why is my camera not connected?  I can't see anything."

sleep 30

echo -e "\n*** DEAD-RECKONING FROM DOOR BACK TO UNDOCKED ***"

/home/pi/wali_pi5/utils/logMaintenance.py "Dead-Reckoning Back To Undocked Position From The Front Door"

echo -e "\n*** DRIVE BACK TOWARD MASTER FOYER  ***"

cmds/say.sh "I want to go home now."

echo -e "\n*** GOTO x: 0.910, y: 4.528  Q(180 deg about Z) "
ros2 action send_goal /navigate_to_position irobot_create_msgs/action/NavigateToPosition "{achieve_goal_heading: true, max_translation_speed: 0.1, max_rotation_speed: 0.5, goal_pose:{pose:{position:{x: 0.455, y: 4.778, z: 0.0}, orientation:{x: 0,y: 0, z: 1, w: 0}}}}"


echo -e "\n*** DRIVE TO Junction ***"
cmds/say.sh "I'm sure moving like an old robot."

echo -e "\n*** GOTO x: 0.225, y: 1.830  Q(-135 deg about Z) "
ros2 action send_goal /navigate_to_position irobot_create_msgs/action/NavigateToPosition "{achieve_goal_heading: true, max_translation_speed: 0.1, max_rotation_speed: 0.5, goal_pose:{pose:{position:{x: 0.225, y: 1.830, z: 0.0}, orientation:{x: 0,y: 0, z: -0.9238795, w: 0.3826834}}}}"

cmds/say.sh "I feel in the way here.  Moving on."

echo -e "\n*** DRIVE To Dining Area ***"

echo -e "\n*** GOTO x: -0.670, y: 0.670  Q(-45 deg about Z) "
ros2 action send_goal /navigate_to_position irobot_create_msgs/action/NavigateToPosition "{achieve_goal_heading: true, max_translation_speed: 0.1, max_rotation_speed: 0.5, goal_pose:{pose:{position:{x: -0.670, y: 0.670,z: 0.0}, orientation:{x: 0,y: 0, z: -0.3826834, w: 0.9238795}}}}"

echo -e "\n*** DRIVE TO Undocked Position and face dock***"

cmds/say.sh "Almost there."


echo -e "\n*** GOTO x: -0.300, y: 0.0  Q(0 deg about Z) "
ros2 action send_goal /navigate_to_position irobot_create_msgs/action/NavigateToPosition "{achieve_goal_heading: true, max_translation_speed: 0.1, max_rotation_speed: 0.5, goal_pose:{pose:{position:{x: -0.300, y: 0.0,z: 0.0}, orientation:{x: 0,y: 0, z: 0, w: 1}}}}"

cmds/say.sh "I'm going to attempt to dock now"


/home/pi/wali_pi5/utils/logMaintenance.py "Attempting manual docking"
echo -e "\n*** ATTEMPTING MANUAL DOCKING"
ros2 action send_goal /dock irobot_create_msgs/action/Dock "{}"

echo -e "\n*** EXCURSION COMPLETE: DID I MAKE IT ON TO THE DOCK?"
cmds/say.sh "Excursion complete.  Did I Make It On To The Dock?"
/home/pi/wali_pi5/utils/logMaintenance.py "Excursion from undocked to front door and back to dock complete"



