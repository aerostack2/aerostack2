cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME "ignition_assets")
project(${PROJECT_NAME} VERSION 0.2.0)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# IGNITION FORTRESS --> ROS2 GALACTIC
find_package(ignition-gazebo6 REQUIRED)
set(IGN_GAZEBO_VER ${ignition-gazebo6_VERSION_MAJOR})

find_package(ignition-common4 REQUIRED COMPONENTS graphics)
set(IGN_COMMON_VER ${ignition-common4_VERSION_MAJOR})
find_package(ignition-fuel_tools7 REQUIRED)
find_package(ignition-math6 REQUIRED)
set(IGN_MATH_VER ${ignition-math6_VERSION_MAJOR})
find_package(ignition-msgs8 REQUIRED)
find_package(ignition-transport11 REQUIRED)
set(IGN_TRANSPORT_VER ${ignition-transport11_VERSION_MAJOR})
find_package(ignition-plugin1 REQUIRED COMPONENTS loader register)
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})
find_package(ignition-rendering6 REQUIRED)
set(IGN_RENDERING_VER ${ignition-rendering6_VERSION_MAJOR})

# find dependencies
set(PROJECT_DEPENDENCIES
  ament_cmake
  ros_gz_sim
  ros_gz_bridge

  # CUSTOM BRIDGES
  rclcpp
  as2_core
  sensor_msgs
  geometry_msgs
  tf2_msgs
  tf2_ros
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

# GPS BRIDGE
find_package(ignition-msgs8 REQUIRED)
find_package(ignition-transport11 REQUIRED)

# ============================================================================
# Hooks
ament_environment_hooks("hooks/resource_paths.dsv.in")
ament_environment_hooks("hooks/resource_paths.sh")

# ============================================================================
# CPP
add_executable(gps_bridge src/gps_bridge.cpp)
ament_target_dependencies(gps_bridge
  rclcpp
  ros_gz_bridge
  as2_core
  sensor_msgs
  ignition-msgs8
  ignition-transport11
)

add_executable(ground_truth_bridge src/ground_truth_bridge.cpp)
ament_target_dependencies(ground_truth_bridge
  rclcpp
  ros_gz_bridge
  as2_core
  geometry_msgs
  ignition-msgs8
  ignition-transport11
)

add_executable(tf_broadcaster src/tf_broadcaster.cpp)
ament_target_dependencies(tf_broadcaster
  rclcpp
  geometry_msgs
  tf2_msgs
  tf2_ros
)

install(TARGETS
  gps_bridge
  ground_truth_bridge
  tf_broadcaster
  DESTINATION lib/${PROJECT_NAME})

# ============================================================================
# Plugins
add_library(SuctionGripper SHARED
  plugins/SuctionGripper.cc
)
target_link_libraries(SuctionGripper PUBLIC
  ignition-gazebo${IGN_GAZEBO_VER}::core
  ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  ignition-rendering${IGN_RENDERING_VER}::ignition-rendering${IGN_RENDERING_VER}
)

install(
  TARGETS SuctionGripper
  DESTINATION lib)

# ============================================================================
# Python modules
ament_python_install_package(
  ign_assets
  PACKAGE_DIR src/ign_assets
)

# ============================================================================
# Launchers
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# ============================================================================
# Resources
install(DIRECTORY
  worlds
  models
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  scripts
  DESTINATION share/${PROJECT_NAME}
)

#============================================================================

ament_package()
