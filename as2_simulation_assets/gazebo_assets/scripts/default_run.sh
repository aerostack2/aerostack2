#!/bin/bash

config_path="$1"
config_path=${config_path:="none"}

SCRIPT_PATH="${AEROSTACK2_PATH}/simulation/gazebo_assets/scripts"
PX4_FOLDER="${AEROSTACK2_WORKSPACE}/src/thirdparty/PX4-Autopilot"

AS2_MODELS="${AEROSTACK2_PATH}/simulation/gazebo_assets/models"
AS2_WORLDS="${AEROSTACK2_PATH}/simulation/gazebo_assets/worlds"

(cd $PX4_FOLDER; DONT_RUN=1 make px4_sitl_rtps gazebo)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$PX4_FOLDER:$PX4_FOLDER/Tools/sitl_gazebo
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$AS2_MODELS
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:$AS2_WORLDS

$SCRIPT_PATH/run_sitl.sh "$PX4_FOLDER/build/px4_sitl_rtps/bin/px4" "$config_path" $PX4_FOLDER $PX4_FOLDER/build/px4_sitl_rtps
