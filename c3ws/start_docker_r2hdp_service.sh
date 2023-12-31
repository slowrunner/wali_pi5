#!/bin/bash
echo -e "Starting Docker Service - 60s wait"
sudo systemctl start docker.r2hdp
# echo "q" | systemctl status docker.r2hdp
echo -e "waiting for docker to start"
sleep 10
docker ps
