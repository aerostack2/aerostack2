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
  ament_cmake_ros
  rclcpp
  rclcpp_components
  pluginlib
  as2_core
  as2_msgs
  as2_behavior
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2_ros
)

# find_package(backward_ros REQUIRED)

# Find dependencies
foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

# Include directories
include_directories(
  include
  include/${PROJECT_NAME}
)

##  Create plugin base library
set(PLUGIN_BASE_FILES
  include/${PROJECT_NAME}/path_planner_plugin_base.hpp
)

add_library(${PROJECT_NAME}_plugin_base SHARED ${PLUGIN_BASE_FILES})

# Specify the language for the target as C++ (as it is a header only library)
set_target_properties(${PROJECT_NAME}_plugin_base PROPERTIES LINKER_LANGUAGE CXX)
ament_target_dependencies(${PROJECT_NAME}_plugin_base ${PROJECT_DEPENDENCIES})

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_plugin_base)
ament_export_targets(${PROJECT_NAME}_plugin_base)

# Library for composing the behavior
add_library(path_planner_component SHARED
  src/path_planner_behavior.cpp
)
ament_target_dependencies(path_planner_component ${PROJECT_DEPENDENCIES})
rclcpp_components_register_nodes(path_planner_component "PathPlannerBehavior")

install(
  TARGETS
    ${PROJECT_NAME}_plugin_base
    path_planner_component
  EXPORT ${PROJECT_NAME}_plugin_base
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

set(PLUGIN_LIST
  a_star
)

foreach(PLUGIN ${PLUGIN_LIST})
  add_subdirectory(plugins/${PLUGIN})
endforeach()

pluginlib_export_plugin_description_file(
  ${PROJECT_NAME}
  plugins.xml
)

# Register component_node
rclcpp_components_register_nodes(path_planner_component "PathPlannerBehavior")

## CREATE BEHAVIOR NODE

set(SOURCE_CPP_FILES
  src/path_planner_behavior.cpp
  src/path_planner_behavior_node.cpp
)

add_executable(${PROJECT_NAME}_node ${SOURCE_CPP_FILES})
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_plugin_base)
ament_target_dependencies(${PROJECT_NAME}_node ${PROJECT_DEPENDENCIES})

target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(${EXECUTABLE_DEPENDENCIES})
ament_export_include_directories(include)

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
