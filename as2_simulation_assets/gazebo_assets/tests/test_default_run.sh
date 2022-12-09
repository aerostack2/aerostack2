#!/bin/bash

# Changing PX4 GPS origin
export PX4_HOME_LAT=28.143971
export PX4_HOME_LON=-16.503213
export PX4_HOME_ALT=0

${AEROSTACK2_STACK}/simulation/gazebo_assets/scripts/default_run.sh "config.json"
