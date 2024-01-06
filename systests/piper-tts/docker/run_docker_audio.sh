#!/bin/bash

docker run -it --rm --device /dev/snd -v /home/pi:/home/pi/ -w /home/pi/wali_pi5/systests/piper-tts/docker audio:0.0.1
