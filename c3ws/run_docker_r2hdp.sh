#!/bin/bash

cd ~/wali_pi5/c3ws
# --rm removes container after exit
# --w working dir to start in
echo -e "\n*** STARTING ROS 2 HUMBLE PLUS IN DOCKER ***"
docker run -it --net=host  -v /home/pi:/home/pi -v /dev/input:/dev/input \
 -v /dev/bus/usb:/dev/bus/usb  -w /home/pi/wali_pi5/c3ws --privileged --rm r2hdp
echo -e "\n*** EXITED DOCKER ***"
