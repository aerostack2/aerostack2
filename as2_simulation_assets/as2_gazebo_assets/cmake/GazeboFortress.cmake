# GazeboFortress.cmake
# Configuration for Gazebo Fortress (ignition-gazebo6)
# Compatible with ROS 2 Humble

set(GZ_SIM "ignition-gazebo6")
set(GZ_COMMON "ignition-common4")
set(GZ_MATH "ignition-math6")
set(GZ_TRANSPORT "ignition-transport11")
set(GZ_PLUGIN "ignition-plugin1")
set(GZ_RENDERING "ignition-rendering6")
set(GZ_MSGS "ignition-msgs8")

# Plugin source directories
set(GAZEBO_PLUGIN_BASE_DIR "plugins/fortress")
set(INDI_CONTROL_PLUGIN_DIR "${GAZEBO_PLUGIN_BASE_DIR}/indi-control")
set(ODOMETRY_PUBLISHER_PLUGIN_DIR "${GAZEBO_PLUGIN_BASE_DIR}/odometry_publisher")

message(STATUS "Using Gazebo Fortress (Ignition) libraries")
