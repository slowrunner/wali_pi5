#!/bin/bash

# ros2 topic echo --once --flow-style /color/mobilenet_detections 

while [ 1 ]
do
    result=$(ros2 topic echo --once --field detections /color/yolov4_detections | \
        cut -d',' -f4-5,7-100)
    echo -e "\n** detections: "
    echo $result | sed -e $'s/class_id/\\\n\\nclass_id/g' | sed -e $'s/),/\\\n/g' | grep class_id
    # sleep 1
done
