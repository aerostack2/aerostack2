#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --
# source "${HOME}/aerostack2_ws/install/setup.bash"
# source "${HOME}/aerostack2_ws/.env"
exec "$@"