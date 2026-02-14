# GazeboHarmonic.cmake
# Configuration for Gazebo Harmonic (gz-sim8)
# Compatible with ROS 2 Jazzy

set(GZ_SIM "gz-sim8")
set(GZ_COMMON "gz-common5")
set(GZ_MATH "gz-math7")
set(GZ_TRANSPORT "gz-transport13")
set(GZ_PLUGIN "gz-plugin2")
set(GZ_RENDERING "gz-rendering8")
set(GZ_MSGS "gz-msgs10")

# Plugin source directories
set(GAZEBO_PLUGIN_BASE_DIR "plugins/harmonic")
set(INDI_CONTROL_PLUGIN_DIR "${GAZEBO_PLUGIN_BASE_DIR}/indi-control")
set(ODOMETRY_PUBLISHER_PLUGIN_DIR "${GAZEBO_PLUGIN_BASE_DIR}/odometry_publisher")

message(STATUS "Using Gazebo Harmonic libraries")
