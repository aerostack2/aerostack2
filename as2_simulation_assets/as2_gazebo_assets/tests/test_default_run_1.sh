#!/bin/bash

# Parsing cli
while [[ "$1" =~ ^- && ! "$1" == "--" ]]; do case $1 in
  -h | --help )
    echo usage: $0 [-h] [--fpv]
    exit 1
    ;;
  --fpv )
    fpv=1
    ;;
esac; shift; done
if [[ "$1" == '--' ]]; then shift; fi

# Chosing drone model from command line input
if [[ "$fpv" == "1" ]]; then
	UAV_MODEL="iris_fpv"
else
	UAV_MODEL="iris"
fi

# Changing PX4 GPS origin
export PX4_HOME_LAT=28.143971
export PX4_HOME_LON=-16.503213
export PX4_HOME_ALT=0

# Set follow mode
export PX4_FOLLOW_MODE=1

# Set world
export PX4_SITL_WORLD="frames"

# Set drone
export UAV_MODEL
export UAV_X=1.0
export UAV_Y=2.0
export UAV_Z=0.0
export UAV_YAW=1.0

${AS2_GZ_ASSETS_SCRIPT_PATH}/default_run.sh
