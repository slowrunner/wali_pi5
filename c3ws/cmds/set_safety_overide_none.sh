#!/bin/bash

# FILE:  cmds/set_safety_overide_none.sh

echo -e "SET SAFETY OVERRIDE NONE"
ros2 param set /motion_control safety_override none

