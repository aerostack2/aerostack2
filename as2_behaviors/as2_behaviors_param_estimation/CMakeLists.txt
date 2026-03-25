# Set Project name and version
cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME as2_behaviors_param_estimation)
project(${PROJECT_NAME} VERSION 1.0.0)

# Default to C++17
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
  rclcpp_action
  std_msgs
  std_srvs
  as2_core
  as2_msgs
  as2_behavior
)

# Find dependencies
foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

# List of behaviors
set(BEHAVIORS_LIST
mass_estimation_behavior
force_estimation_behavior
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
