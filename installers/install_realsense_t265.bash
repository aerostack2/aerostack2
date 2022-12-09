#!/bin/bash

sudo apt install ros-${ROS_DISTRO}-realsense2-camera* &&
sudo wget https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules --directory /etc/udev/rules.d/ 




