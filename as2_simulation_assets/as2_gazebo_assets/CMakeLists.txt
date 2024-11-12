cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME "as2_gazebo_assets")
project(${PROJECT_NAME} VERSION 1.0.0)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# IGNITION FORTRESS --> ROS2 GALACTIC
# find_package(gz-gazebo6 REQUIRED)
# set(IGN_GAZEBO_VER ${gz-gazebo6_VERSION_MAJOR})

#by default use the fortress version of ignition
set(GZ_VERSION "fortress")

# check if harmonic libs are available
find_package(gz-common5 QUIET)
if(gz-common5_FOUND)
  set(GZ_VERSION "harmonic")
endif()

message("Using GAZEBO version: ${GZ_VERSION}")

if( ${GZ_VERSION} STREQUAL "harmonic" )

set(GZ_COMMON "gz-common5")
set(GZ_MATH "gz-math7")
set(GZ_TRANSPORT "gz-transport13")
set(GZ_PLUGIN "gz-plugin2")
set(GZ_RENDERING "gz-rendering8")
set(GZ_MSGS "gz-msgs10")

elseif( ${GZ_VERSION} STREQUAL "fortress" )

set(GZ_COMMON "ignition-common4")
set(GZ_MATH "ignition-math6")
set(GZ_TRANSPORT "ignition-transport11")
set(GZ_PLUGIN "ignition-plugin1")
set(GZ_RENDERING "ignition-rendering6")
set(GZ_MSGS "ignition-msgs8")

endif()



find_package(${GZ_COMMON} REQUIRED COMPONENTS graphics)
find_package(${GZ_MATH} REQUIRED)
find_package(${GZ_TRANSPORT} REQUIRED)
find_package(${GZ_PLUGIN} REQUIRED COMPONENTS loader register)
find_package(${GZ_RENDERING} REQUIRED)
find_package(${GZ_MSGS} REQUIRED)
# find_package(backward_ros REQUIRED)


# find dependencies
set(PROJECT_DEPENDENCIES
  ament_cmake
  ament_cmake_python
  ros_gz_sim
  ros_gz_bridge

  # CUSTOM BRIDGES
  rclcpp
  as2_core
  as2_msgs
  sensor_msgs
  geometry_msgs
  tf2_msgs
  tf2_ros
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

include_directories(include include/${PROJECT_NAME})


# ============================================================================
# Hooks
ament_environment_hooks("hooks/resource_paths.dsv.in")
ament_environment_hooks("hooks/resource_paths.sh")

# ============================================================================
# CPP
add_executable(gps_bridge src/gps_bridge/gps_bridge.cpp src/gps_bridge/gps_bridge_node.cpp)
ament_target_dependencies(gps_bridge
  rclcpp
  ros_gz_bridge
  as2_core
  sensor_msgs
  ${GZ_MSGS}
  ${GZ_TRANSPORT}
)

add_executable(ground_truth_bridge src/ground_truth_bridge/ground_truth_bridge.cpp src/ground_truth_bridge/ground_truth_bridge_node.cpp)
ament_target_dependencies(ground_truth_bridge
  rclcpp
  ros_gz_bridge
  as2_core
  geometry_msgs
  ${GZ_MSGS}
  ${GZ_TRANSPORT}
)

add_executable(azimuth_bridge src/azimuth_bridge/azimuth_bridge.cpp src/azimuth_bridge/azimuth_bridge_node.cpp)
ament_target_dependencies(azimuth_bridge
  rclcpp
  ${GZ_MSGS}
  ${GZ_TRANSPORT}
  ros_gz_bridge
  std_msgs
  as2_core
  sensor_msgs
  ${GZ_MSGS}
  ${GZ_TRANSPORT}
)

add_executable(object_tf_broadcaster src/object_tf_broadcaster/object_tf_broadcaster.cpp
                                      src/object_tf_broadcaster/object_tf_broadcaster_node.cpp)
ament_target_dependencies(object_tf_broadcaster
  rclcpp
  geometry_msgs
  tf2_msgs
  tf2_ros
  ros_gz_bridge
  ${GZ_MSGS}
  ${GZ_TRANSPORT}
)

add_executable(gimbal_bridge src/gimbal_bridge/gimbal_bridge.cpp src/gimbal_bridge/gimbal_bridge_node.cpp)
ament_target_dependencies(gimbal_bridge
  rclcpp
  ros_gz_bridge
  std_msgs
  as2_core
  as2_msgs
  geometry_msgs
  ${GZ_MSGS}
  ${GZ_TRANSPORT}
)



install(TARGETS
  azimuth_bridge
  gps_bridge
  ground_truth_bridge
  object_tf_broadcaster
  gimbal_bridge
  DESTINATION lib/${PROJECT_NAME})

# TODO(miferco97): Add the plugins to the CMakeLists.txt

# ============================================================================
# Plugins
# add_library(SuctionGripper SHARED
#   plugins/SuctionGripper.cc
# )
# target_link_libraries(SuctionGripper PUBLIC
#   gz-gazebo${IGN_GAZEBO_VER}::core
#   gz-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
#   gz-rendering${IGN_RENDERING_VER}::ignition-rendering${IGN_RENDERING_VER}
# )

# install(
#   TARGETS SuctionGripper
#   DESTINATION lib)

# ============================================================================
# Python modules
ament_python_install_package(
  as2_gazebo_assets
  PACKAGE_DIR src/as2_gazebo_assets

)

# ============================================================================
# Include
install(
  DIRECTORY include/
  DESTINATION include
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

# ============================================================================
# Tests

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  set(_pytest_tests
    src/as2_gazebo_assets/tests/test_models.py
    test/test_copyright.py
    # Add other test files here
  )
  foreach(_test_path ${_pytest_tests})
    get_filename_component(_test_name ${_test_path} NAME_WE)
    ament_add_pytest_test(${_test_name} ${_test_path}
      APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
      TIMEOUT 60
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )
  endforeach()
endif()

ament_package()
