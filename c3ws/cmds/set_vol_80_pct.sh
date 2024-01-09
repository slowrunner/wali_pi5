#!/bin/bash

amixer -D pulse sset Master 80%
~/wali_pi5/systests/piper-tts/piper.sh 'Volume set to 80 percent' 
