#!/bin/bash

# FILE:  nap_pi5_for.sh
# USAGE:  ./nap_pi5_for.sh [N.n hours]
# REQ: sudo -E rpi-eeprom-onfig --edit, add POWER_OFF_ON_HALT=1 and WAKE_ON_GPIO=0

if [ "$#" -ne 1 ] ;
	then echo "Usage:  ./nap_pi5_for.sh NN.n (NN.n hours) "
	exit
fi
echo "WaLiPi5 is going to nap for $1 hours"
~/wali_pi5/utils/logMaintenance.py 'WaLiPi5 is going to nap for '$1' hours

# Convert hours to seconds to set alarm'
naptime="+"$(( $1 * 3600 ))
echo $naptime | sudo tee /sys/class/rtc/rtc0/wakealarm
sudo shutdown -h +1

