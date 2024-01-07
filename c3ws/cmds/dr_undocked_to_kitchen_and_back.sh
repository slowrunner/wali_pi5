#!/bin/bash

cd /home/pi/wali_pi5/c3ws

cmds/say.sh "I'm headed to the kitchen now"

echo -e "\n*** DEAD-RECKONING TO KITCHEN FROM UNDOCKED AND BACK***"

/home/pi/wali_pi5/utils/logMaintenance.py "Dead-Reckoning To Kitchen From Undocked (and back)"

echo -e "\n*** GOTO x: -0.670, y: 0.670  Q(45 deg about Z) "
ros2 action send_goal /navigate_to_position irobot_create_msgs/action/NavigateToPosition "{achieve_goal_heading: true, max_translation_speed: 0.1, max_rotation_speed: 0.5, goal_pose:{pose:{position:{x: -0.670, y: 0.670,z: 0.0}, orientation:{x: 0,y: 0, z: 0.3826834, w: 0.9238795}}}}"

cmds/say.sh "That was step 1"

echo -e "\n*** DRIVE TO KITCHEN 2 ***"

echo -e "\n*** GOTO x: 0.225, y: 1.830  Q(-45 deg about Z) "
ros2 action send_goal /navigate_to_position irobot_create_msgs/action/NavigateToPosition "{achieve_goal_heading: true, max_translation_speed: 0.1, max_rotation_speed: 0.5, goal_pose:{pose:{position:{x: 0.225, y: 1.830, z: 0.0}, orientation:{x: 0,y: 0, z: -0.3826834, w: 0.9238795}}}}"

echo -e "\n*** DRIVE TO KITCHEN 3 ***"

cmds/say.sh "On to the breakfast area"

echo -e "\n*** GOTO x: 1.456, y: 0.455  Q(45 deg about Z) "
ros2 action send_goal /navigate_to_position irobot_create_msgs/action/NavigateToPosition "{achieve_goal_heading: true, max_translation_speed: 0.1, max_rotation_speed: 0.5, goal_pose:{pose:{position:{x: 1.456, y: 0.455, z: 0.0}, orientation:{x: 0,y: 0, z: 0.3826834, w: 0.9238795}}}}"

echo -e "\n*** DRIVE TO KITCHEN 4 ***"
cmds/say.sh "And finally to the kitchen"


echo -e "\n*** GOTO x: 2.957, y: 1.600  Q(-135 deg about Z) "
ros2 action send_goal /navigate_to_position irobot_create_msgs/action/NavigateToPosition "{achieve_goal_heading: true, max_translation_speed: 0.1, max_rotation_speed: 0.5, goal_pose:{pose:{position:{x: 2.957, y: 1.600, z: 0.0}, orientation:{x: 0,y: 0, z: -0.9238795, w: 0.3826834}}}}"

/home/pi/wali_pi5/utils/logMaintenance.py "Arrived at Kitchen"

cmds/say.sh "Look at me!  Made it all the way to the kitchen by my self."

sleep 30

echo -e "\n*** DEAD-RECKONING TO UNDOCKED FROM KITCHEN BACK TO UNDOCKED ***"

/home/pi/wali_pi5/utils/logMaintenance.py "Dead-Reckoning Back To Undocked position From Kitchen"

echo -e "\n*** DRIVE TO Nook  ***"

cmds/say.sh "I want to go home now."

echo -e "\n*** GOTO x: 1.456, y: 0.455  Q(135 deg about Z) "
ros2 action send_goal /navigate_to_position irobot_create_msgs/action/NavigateToPosition "{achieve_goal_heading: true, max_translation_speed: 0.1, max_rotation_speed: 0.5, goal_pose:{pose:{position:{x: 1.456, y: 0.455, z: 0.0}, orientation:{x: 0,y: 0, z: 0.9238795, w: 0.3826834}}}}"


echo -e "\n*** DRIVE TO Junction ***"
cmds/say.sh "There has to be a shorter way than this."

echo -e "\n*** GOTO x: 0.225, y: 1.830  Q(-135 deg about Z) "
ros2 action send_goal /navigate_to_position irobot_create_msgs/action/NavigateToPosition "{achieve_goal_heading: true, max_translation_speed: 0.1, max_rotation_speed: 0.5, goal_pose:{pose:{position:{x: 0.225, y: 1.830, z: 0.0}, orientation:{x: 0,y: 0, z: -0.9238795, w: 0.3826834}}}}"

cmds/say.sh "I'm sort of in the way here.  Moving on."

echo -e "\n*** DRIVE To Dining Area ***"

echo -e "\n*** GOTO x: -0.670, y: 0.670  Q(-45 deg about Z) "
ros2 action send_goal /navigate_to_position irobot_create_msgs/action/NavigateToPosition "{achieve_goal_heading: true, max_translation_speed: 0.1, max_rotation_speed: 0.5, goal_pose:{pose:{position:{x: -0.670, y: 0.670,z: 0.0}, orientation:{x: 0,y: 0, z: -0.3826834, w: 0.9238795}}}}"

echo -e "\n*** DRIVE TO Undocked Position ***"

cmds/say.sh "Almost there."


echo -e "\n*** GOTO x: -0.300, y: 0.0  Q(-180 deg about Z) "
ros2 action send_goal /navigate_to_position irobot_create_msgs/action/NavigateToPosition "{achieve_goal_heading: true, max_translation_speed: 0.1, max_rotation_speed: 0.5, goal_pose:{pose:{position:{x: -0.300, y: 0.0,z: 0.0}, orientation:{x: 0,y: 0, z: -1.0, w: 0.0}}}}"

cmds/say.sh "Home again.  I think I'll stay here for a while."


/home/pi/wali_pi5/utils/logMaintenance.py "Arrived Back at Undocked Position"
