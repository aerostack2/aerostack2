#!/bin/bash

# Set world
export UAV_WORLD="empty"

# Set drone
export UAV_MODEL="quadrotor_base"
export UAV_X=1.0
export UAV_Y=2.0
export UAV_Z=0.0
export UAV_YAW=1.0

export RUN_ON_START=1

${AEROSTACK2_PATH}/simulation/ignition_assets/scripts/run_ign.sh