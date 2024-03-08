# Set the minimum required CMake version
cmake_minimum_required(VERSION 3.5)

# Set the project name
set(PROJECT_NAME as2_behaviors_perception)
project(${PROJECT_NAME} VERSION 1.0.0)

# Default to C++17 if not set
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Set Release as default
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Set fPIC to ON by default
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Set commons dependencies
set(PROJECT_DEPENDENCIES
  ament_cmake
  rclcpp
  std_msgs
  std_srvs
  as2_core
  as2_msgs
  as2_behavior
  OpenCV
  cv_bridge
)

# Find dependencies
foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

# List of behaviors
set(BEHAVIORS_LIST
  detect_aruco_markers_behavior
  point_gimbal_behavior
)

# Add subdirectories
foreach(BEHAVIOR ${BEHAVIORS_LIST})
  add_subdirectory(${BEHAVIOR})
endforeach()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${PROJECT_DEPENDENCIES})

ament_package()