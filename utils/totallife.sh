#!/bin/bash
#
# totallife.sh    print total hours and sessions of life in life.log
#
# requires bc  (sudo apt-get install bc)
#
echo "(Cleaning life.log first)"
/home/pi/wali_pi5/plib/cleanlifelog.py
echo " "
fn="/home/pi/wali_pi5/logs/life.log"
totalLife=`(awk -F':' '{sum+=$3}END{print sum;}' $fn)`
echo "*** Create3-WALI TOTAL LIFE STATISTICS ***"
echo "Total Life: " $totalLife "hrs (since Dec 11, 2023)"
echo "Playtimes (Undocked-Docked):" `(grep -c " Docking: success " $fn)`
booted=`(grep -c "\- boot \-" $fn)`
echo "Sessions (boot): " `(grep -c "\- boot \-" $fn)`
aveSession=`(echo "scale=1; ($totalLife / $booted)" | bc -l)`
echo "Average Session: " $aveSession "hrs"
safetyShutdowns=`(grep -c "safety shutdown" $fn)`
echo "Safety Shutdowns: " $safetyShutdowns 
