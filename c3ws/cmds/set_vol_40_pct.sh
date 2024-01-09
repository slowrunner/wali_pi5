#!/bin/bash

amixer -D pulse sset Master 40%
~/wali_pi5/systests/piper-tts/piper.sh 'Volume set to 40 percent'
