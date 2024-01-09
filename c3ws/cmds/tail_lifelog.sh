#!/bin/bash
/home/pi/wali_pi5/utils/totallife.sh
echo -e "\n*** TAIL LIFE LOG ***"
tail -20 /home/pi/wali_pi5/logs/life.log
