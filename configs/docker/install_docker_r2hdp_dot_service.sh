#!/bin/bash

cd ~/wali_pi5/configs/docker
sudo cp etc_systemd_system-docker.r2hdp.service /etc/systemd/system/docker.r2hdp.service
sudo systemctl enable docker.r2hdp
sudo systemctl start docker.r2hdp
