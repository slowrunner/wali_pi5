#!/bin/bash
#
# totallife.sh    print total hours and sessions of life in life.log
#
# requires bc  (sudo apt-get install bc)
#
# Counted Keys:
#   Playtimes:           " Docking: success "
#   Sessions:            "- boot -"              ( "\- boot \-" search string )
#   Safety shutdowns:    "safety shutdown"

echo "(Cleaning life.log first)"
/home/pi/wali_pi5/plib/cleanlifelog.py
echo " "
fn="/home/pi/wali_pi5/logs/life.log"
ofn='/home/pi/wali_pi5/logs/odometer.log'
# fn="/home/pi/wali_pi5/utils/test_life.log"
totalAwake=`(awk -F':' '{sum+=$3}END{print sum;}' $fn)`
totalNaps=`(awk -F'nap for' '{sum+=$2}END{print sum;}' $fn)`
totalLife=`(echo "scale=1; ($totalAwake + $totalNaps)" | bc)`
echo "*** Create3-WALI TOTAL LIFE STATISTICS ***"
echo "Total Awake: " $totalAwake " hrs"
echo "Total Naps:    " $totalNaps " hrs"
echo "Total Life: " $totalLife "hrs (since Dec 11, 2023)"
echo "Playtimes (Undocked-Docked):" `(grep -c " Docking: success " $fn)`
booted=`(grep -c "\- boot \-" $fn)`
echo "Sessions (boot): " `(grep -c "\- boot \-" $fn)`
aveSession=`(echo "scale=1; ($totalAwake / $booted)" | bc -l)`
echo "Average Session: " $aveSession "hrs"
safetyShutdowns=`(grep -c "safety shutdown" $fn)`
echo "Safety Shutdowns: " $safetyShutdowns 
totalMoved=`(awk -F'moved:' '{sum+=$2}END{printf "%.1f", sum;}' $ofn)`
totalMovedFt=`(echo "scale=1; ($totalMoved / 0.3048)" | bc)`
echo "Total Travel: " $totalMoved "meters" $totalMovedFt "feet"

