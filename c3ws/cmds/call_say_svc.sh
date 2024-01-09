#!/bin/bash

if [ "$#" -ne 1 ] ;
	then echo 'Usage:  ./call_say_svc.sh "string to speak" '
	exit
fi

ros2 service call /say wali_interfaces/srv/Say "saystring:  '${1}'"
