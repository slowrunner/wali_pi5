#!/bin/bash

# FILE:  cmds/set_safety_overide_backup.sh
#  Leaves operational
#   - Max Speed Limit: 0.306 m/s
#   - Acceleration Limit: 900 mm/s^2
#   - cliff sensing
#   - kidnap sensing

echo -e "SET SAFETY OVERRIDE (ONLY) BACKUP"

# ros2 param get /motion_control reflexes.REFLEX_BUMP
ros2 param set /motion_control safety_override backup_only

