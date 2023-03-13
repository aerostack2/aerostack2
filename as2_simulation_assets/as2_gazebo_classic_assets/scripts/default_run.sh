#!/bin/bash

config_path="$1"
config_path=${config_path:="none"}

# If ${PX4_FOLDER} is not set, then set it to "${AEROSTACK2_WORKSPACE}/src/thirdparty/PX4-Autopilot"
if [ -z ${PX4_FOLDER+x} ]; then
    echo "PX4_FOLDER is unset, setting it to ${AEROSTACK2_WORKSPACE}/src/thirdparty/PX4-Autopilot"
    export PX4_FOLDER="${AEROSTACK2_WORKSPACE}/src/thirdparty/PX4-Autopilot"
fi
echo "PX4_FOLDER is set to '$PX4_FOLDER'"


(cd ${PX4_FOLDER}; DONT_RUN=1 make px4_sitl gazebo-classic -j 1)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${PX4_FOLDER}:${PX4_FOLDER}/Tools/simulation/gazebo-classic

$AS2_GZ_ASSETS_SCRIPT_PATH/run_sitl.sh "$config_path"
