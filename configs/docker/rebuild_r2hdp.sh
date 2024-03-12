#!/bin/bash

# FILE: rebuild_r2hdp.sh

# Will delete all docker images and containers and build cache
# then rebuild r2hdp docker image

echo -e "\n*** STARTING DOCKER r2hdp REBUILD"
echo -e `date +"%A %D %H:%M:%S"`
cd ~/wali_pi5/configs/docker

echo -e "\n- DISK SPACE"
./du_docker.sh

echo -e "\n- IMAGES"
./list_images.sh

echo -e "\n- DELETE DOCKER IMAGES AND BUILD CACHE"
docker system prune -af
# ./delete_docker_images.sh

# echo -e "\n- PRUNE BUILD CACHE"
# ./prune_build_cache.sh

echo -e "\n- BUILD DOCKER IMAGE r2hdp"
# build a ros humble desktop plus navigation, slam-toolbox, localization, rtabmap, depthai, etc.,  tagged as r2hdp
sudo docker build --no-cache . -t r2hdp -f docker_files/ros2_humble_desktop_plus_dockerfile

echo -e "\n- COMPLETED: BUILD DOCKER IMAGE r2hdp"
echo -e `date +"%A %D %H:%M:%S"`

echo -e "\n- DISK SPACE"
./du_docker.sh

echo -e "\n- IMAGES"
./list_images.sh

echo -e "\n\n********* exiting rebuild_r2hdp.sh *************\n"

