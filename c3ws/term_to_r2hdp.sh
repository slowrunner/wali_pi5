#!/bin/bash

# start a term to already running ROS 2 Humble Desktop Plus Docker container named r2hdp
docker ps
echo -e "\nStarting BASH TERMINAL to r2hdp DOCKER container"
echo -e "To exit:  exit (will not terminate container)"
docker exec -it r2hdp bash
echo -e "Exited Docker Terminal"
