#!/bin/bash


# pushd /home/pi/wali_pi5/systests/piper-tts
echo $1 | piper   --model /home/pi/wali_pi5/c3ws/models/piper-tts/en_US-arctic-medium.onnx    --output_raw | aplay -D plughw:2,0 -r 22050 -f S16_LE -t raw - 
# popd
