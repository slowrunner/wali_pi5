#!/bin/bash

usage="\n*** USAGE: cmds/follow_wall.sh left|right nSeconds ***\n"

if [ "$#" -ne 2 ] ;
    then echo -e $usage
         exit
fi

if [ $1 == "left" ] ;
    then dir=1
    else if [ $1 == "right" ] ;
            then dir=-1
            else echo -e $usage
            exit
         fi
fi

echo -e "Wall Follow ${1} for ${2} seconds"

ros2 action send_goal /wall_follow irobot_create_msgs/action/WallFollow "{follow_side: ${dir}, max_runtime: {sec: ${2}, nanosec: 0}}"
