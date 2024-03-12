#!/bin/bash

pushd /home/pi/wali_pi4/dai_ws
./src/depthai-ros/build.sh

echo -e "\n be sure to source install/setup.bash"
popd
