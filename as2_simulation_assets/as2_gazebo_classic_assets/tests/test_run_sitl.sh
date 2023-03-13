#!/bin/bash

PX4_FOLDER="${AEROSTACK2_WORKSPACE}/src/thirdparty/PX4-Autopilot"
(cd $PX4_FOLDER; DONT_RUN=1 make px4_sitl gazebo-classic)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$PX4_FOLDER:$PX4_FOLDER/Tools/sitl_gazebo

$AS2_GZ_ASSETS_SCRIPT_PATH/run_sitl.sh "$PX4_FOLDER/build/px4_sitl_default/bin/px4" "config.json" $PX4_FOLDER $PX4_FOLDER/build/px4_sitl_default

