#!/bin/bash

docker run -it \
 -v /dev/:/dev/  \
 -v /home/pi:/home/pi \
 -e TZ=America/New_York \
 -w /home/pi/wali_pi5/systests/Oak-D-Lite/dockerized \
 --net=host \
 --privileged \
 luxonis/depthai-ros:humble-latest bash
