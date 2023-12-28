#!/bin/bash
echo -e "\n*** CLEAN LIFE LOG ***"
/home/pi/wali_pi5/plib/cleanlifelog.py
echo -e "\n*** TAIL LIFE LOG ***"
tail -10 /home/pi/wali_pi5/logs/life.log
