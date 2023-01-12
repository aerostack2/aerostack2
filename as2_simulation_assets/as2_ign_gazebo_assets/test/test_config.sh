#!/bin/bash

export RUN_ON_START=1

config=${AEROSTACK2_PATH}/simulation/as2_ign_gazebo_assets/test/config.json

${AEROSTACK2_PATH}/simulation/as2_ign_gazebo_assets/scripts/run_ign.sh ${config}