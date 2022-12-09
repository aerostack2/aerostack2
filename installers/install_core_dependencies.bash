#!/bin/bash

if [[ -z "$ROS_DISTRO" ]]; then
    echo ""; echo "[ERROR] ROS environment is not set. (ROS_DISTRO env variable empty) Exiting.."
    exit 1
fi

sudo apt update
sudo apt install tmux python3-vcstool python3-rosdep python3-pip python3-colcon-common-extensions -y
