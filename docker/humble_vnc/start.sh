#!/bin/sh
path_to_aerostack2=$(echo $PWD | sed 's/\(.*\)\/src\/aerostack2\/docker\/Humble/\1/')

docker build -t ros2:humble_aerostack2 --shm-size=512m - < $PWD/dockerfile
docker run --name=ros2_humble_aerostack2 --volume=$path_to_aerostack2:/home/ubuntu/ros2/aerostack2_ws \
 --network=bridge -p 6080:80 --shm-size=512m \
 --security-opt seccomp=unconfined \
 --cap-add IPC_OWNER \
 ros2:humble_aerostack2