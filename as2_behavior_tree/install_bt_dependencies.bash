#!/bin/bash

sudo apt-get update
sudo apt-get install ros-$ROS_DISTRO-behaviortree-cpp-v3 -y 
sudo apt-get install ros-$ROS_DISTRO-nav2-msgs -y 
sudo apt-get install ros-$ROS_DISTRO-nav2-behavior-tree -y 
