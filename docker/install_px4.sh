#!/bin/bash

# Export PX4_FOLDER
PX4_FOLDER=${AEROSTACK2_PATH}/thirdparties/px4/thirdparties/PX4-Autopilot
echo "export PX4_FOLDER=${PX4_FOLDER}" >> /home/cvar/.bashrc
echo "export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/usr/share/gazebo-11" >> /home/cvar/.bashrc

# Dependencies for PX4
bash ${PX4_FOLDER}/Tools/setup/ubuntu.sh --no-sim-tools --no-nuttx

# Dependencies simulation gazebo 11
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install -y --quiet gazebo11 libgazebo11-dev dmidecode gstreamer1.0-plugins-bad gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly gstreamer1.0-libav libeigen3-dev libgstreamer-plugins-base1.0-dev libimage-exiftool-perl libopencv-dev libxml2-utils pkg-config protobuf-compiler

# Compile PX4 SITL
DONT_RUN=1 make px4_sitl gazebo-classic

exit 0