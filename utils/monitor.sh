#!/bin/bash


while true; \
do echo -e "\n********** WaLiPi5  MONITOR ******************************"; \
echo -n `date +"%A %D"`; \
echo ""; \
uptime; \
vcgencmd measure_temp && vcgencmd measure_clock arm && vcgencmd get_throttled; \
free -h; \
docker ps; \
sleep 10; \
echo " "; \
done
