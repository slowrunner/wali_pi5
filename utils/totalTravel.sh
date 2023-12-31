#!/bin/bash


fn='/home/pi/wali_pi5/logs/odometer.log'

echo -e "\n*** TRAVEL FROM ODOMETRY.LOG ***"
totalMoved=`(awk -F'moved:' '{sum+=$2}END{printf "%.1f", sum;}' $fn)`
totalMovedFt=`(echo "scale=1; ($totalMoved / 0.3048)" | bc)`
echo "Total Travel: " $totalMoved "meters" $totalMovedFt "feet"

