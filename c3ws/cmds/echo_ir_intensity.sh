#/bin/bash

echo -e "\n*** ECHO IR INTENSITY"
echo -e "ros2 topic echo --flow-style /ir_intensity"

IFS='='

while [ 1 ]
do
    result=$(ros2 topic echo --once --flow-style --field readings /ir_intensity | cut -d',' -f3-4,7-8,11-12,15-16,19-20,23-24,27-28)
    echo $result | sed -e $'s/frame_id/\\\nframe_id/g'
    # sleep 1
done

