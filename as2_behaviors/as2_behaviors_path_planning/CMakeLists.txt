cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME as2_behaviors_path_planning)
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
  rclcpp_components
)

find_package(backward_ros REQUIRED)

# Find dependencies
foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

# List of behaviors
set(BEHAVIORS_LIST
  path_planner_behavior
)

# Add subdirectories
foreach(BEHAVIOR ${BEHAVIORS_LIST})
  add_subdirectory(${BEHAVIOR})
endforeach()

# Register component_node again in main CMakeLists
# https://robotics.stackexchange.com/questions/97057/ros2-components-registration-from-subdirectory-cmakelists-file/
rclcpp_components_register_nodes(path_planner_component "PathPlannerBehavior")

ament_export_dependencies(${PROJECT_DEPENDENCIES})

if(BUILD_TESTING)
find_package(ament_lint_auto REQUIRED)
  file(GLOB_RECURSE EXCLUDE_FILES
    build/*
    install/*
  )
  set(AMENT_LINT_AUTO_FILE_EXCLUDE ${EXCLUDE_FILES})
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()