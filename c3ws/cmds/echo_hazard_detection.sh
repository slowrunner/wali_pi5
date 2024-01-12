#/bin/bash

echo -e "\n*** ECHO HAZARD_DETECTION"
echo -e "ros2 topic echo --flow-style /hazard_detection"
while [ 1 ]
do
    ros2 topic echo --once --flow-style /hazard_detection
    sleep 2
done

