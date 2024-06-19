#!/bin/bash
export GZ_VERSION="@GZ_VERSION@"
ament_prepend_unique_value GZ_SIM_RESOURCE_PATH "$AMENT_CURRENT_PREFIX/share/as2_ign_gazebo_assets/models"
ament_prepend_unique_value GZ_SIM_RESOURCE_PATH "$AMENT_CURRENT_PREFIX/share/as2_ign_gazebo_assets/worlds"
ament_prepend_unique_value GZ_SIM_SYSTEM_PLUGIN_PATH "$AMENT_CURRENT_PREFIX/lib"

ament_prepend_unique_value LD_LIBRARY_PATH "$AMENT_CURRENT_PREFIX/lib"