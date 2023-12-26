#!/bin/bash

cd ~/wali_pi5/c3ws
# --rm    remove container after running
docker run -it --net=host  -v ~/pi5desk/ros2ws:/ros2ws -w /ros2ws --rm r2hdp
