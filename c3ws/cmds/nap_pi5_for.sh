#!/bin/bash

# FILE:  nap_pi5_for.sh
# USAGE:  ./nap_pi5_for.sh [minutes]
# REQ: sudo -E rpi-eeprom-onfig --edit, add POWER_OFF_ON_HALT=1 and WAKE_ON_GPIO=0

if [ "$#" -ne 1 ] ;
	then echo "Usage:  ./nap_pi5_for.sh NN (NN minutes) "
	exit
fi
echo "WaLiPi5 is going to nap for $1 minutes"
~/wali_pi5/utils/logMaintenance.py 'WaLiP5 is going to nap for '$1' minutes'
naptime="+"$(( $1 * 60 ))
echo $naptime | sudo tee /sys/class/rtc/rtc0/wakealarm
sudo shutdown -h +1

