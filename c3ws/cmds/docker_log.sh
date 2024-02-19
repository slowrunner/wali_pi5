#!/bin/bash

# Continuously print out log from the detached docker container running wali_node+

while true; \
do echo -e "\n********** WaLiPi5  Docker Log ******************************"; \
echo -e `date +"%A %D %H:%M"`; \
dc=$(docker ps); \
dc=${dc#*NAMES}; \
dc=${dc% r2hdp*}; \
dc=${dc% r2hdp*}; \
# echo $dc; \
if [  -f /usr/bin/docker ]; then docker logs $dc;  fi; \
sleep 1; \
echo " "; \
done
