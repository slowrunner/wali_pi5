#!/bin/bash


echo -e "\n*** DEAD-RECKONING TO KITCHEN FROM UNDOCKED ***"

/home/pi/wali_pi5/utils/logMaintenance.py "Dead-Reckoning To Kitchen From Undocked"

echo -e "\n*** GOTO x: -0.670, y: 0.670  Q(45 deg about Z) "
ros2 action send_goal /navigate_to_position irobot_create_msgs/action/NavigateToPosition "{achieve_goal_heading: true, max_translation_speed: 0.1, max_rotation_speed: 0.5, goal_pose:{pose:{position:{x: -0.670, y: 0.670,z: 0.0}, orientation:{x: 0,y: 0, z: 0.3826834, w: 0.9238795}}}}"

echo -e "\n*** DRIVE TO KITCHEN 2 ***"

echo -e "\n*** GOTO x: 0.225, y: 1.830  Q(-45 deg about Z) "
ros2 action send_goal /navigate_to_position irobot_create_msgs/action/NavigateToPosition "{achieve_goal_heading: true, max_translation_speed: 0.1, max_rotation_speed: 0.5, goal_pose:{pose:{position:{x: 0.225, y: 1.830, z: 0.0}, orientation:{x: 0,y: 0, z: -0.3826834, w: 0.9238795}}}}"

echo -e "\n*** DRIVE TO KITCHEN 3 ***"

echo -e "\n*** GOTO x: 1.456, y: 0.455  Q(45 deg about Z) "
ros2 action send_goal /navigate_to_position irobot_create_msgs/action/NavigateToPosition "{achieve_goal_heading: true, max_translation_speed: 0.1, max_rotation_speed: 0.5, goal_pose:{pose:{position:{x: 1.456, y: 0.455, z: 0.0}, orientation:{x: 0,y: 0, z: 0.3826834, w: 0.9238795}}}}"

echo -e "\n*** DRIVE TO KITCHEN 4 ***"

echo -e "\n*** GOTO x: 2.957, y: 1.600  Q(-135 deg about Z) "
ros2 action send_goal /navigate_to_position irobot_create_msgs/action/NavigateToPosition "{achieve_goal_heading: true, max_translation_speed: 0.1, max_rotation_speed: 0.5, goal_pose:{pose:{position:{x: 2.957, y: 1.600, z: 0.0}, orientation:{x: 0,y: 0, z: -0.9238795, w: 0.3826834}}}}"

/home/pi/wali_pi5/utils/logMaintenance.py "Arrived at Kitchen"
