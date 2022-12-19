#!/bin/bash

# Changing PX4 GPS origin
export PX4_HOME_LAT=28.143971
export PX4_HOME_LON=-16.503213
export PX4_HOME_ALT=0

${AS2_GZ_ASSETS_SCRIPT_PATH}/default_run.sh "config.json"
