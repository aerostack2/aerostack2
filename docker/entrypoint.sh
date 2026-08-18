#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --
source "${AEROSTACK2_WORKSPACE}/install/setup.bash"

# TODO: source failing
# source "$AEROSTACK2_PATH/as2_cli/setup_env.bash"
exec "$@"