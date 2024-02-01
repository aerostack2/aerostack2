#!/bin/bash

# If ROS_DISTRO is not humble or galactic, thwo an error
if [ "${ROS_DISTRO}" != "humble" ] && [ "${ROS_DISTRO}" != "galactic" ]; then
    echo "ROS_DISTRO must be humble or galactic"
    exit 1
fi

echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/cvar/.bashrc
echo "export AEROSTACK2_PATH=$AEROSTACK2_PATH" >> /home/cvar/.bashrc
echo "source $AEROSTACK2_PATH/as2_cli/setup_env.bash" >> /home/cvar/.bashrc

if [ "${ROS_DISTRO}" = "galactic" ]; then
   cd ${AEROSTACK2_PATH}/as2_aerial_platforms/as2_platform_ign_gazebo && touch COLCON_IGNORE
   cd ${AEROSTACK2_PATH}/as2_simulation_assets/as2_ign_gazebo_assets && touch COLCON_IGNORE
fi