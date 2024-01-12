#!/bin/bash

# FILE:  cmds/set_safety_overide_full.sh

echo -e "SET SAFETY OVERRIDE FULL"
ros2 param set /motion_control safety_override full

