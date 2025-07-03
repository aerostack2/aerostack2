cmake_minimum_required(VERSION 3.8)
set(PROJECT_NAME  as2_behaviors_gripper)
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
  rclcpp_action
  std_msgs
  as2_core
  as2_msgs
  as2_behavior
  geometry_msgs
)


# Find dependencies
foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

include_directories(
  include
  include/${PROJECT_NAME}
)

set(SRC_FILES
  src/gripper_handler_behavior.cpp
)

# Plugings
set(PLUGINS_CPP_FILES
  plugins/gripper_handler_plugin_gazebo.cpp
)

# Executables
set(NODE_SOURCES
  src/gripper_handler_behavior_node.cpp
)


# Library
add_library(${PROJECT_NAME} SHARED
  ${SRC_FILES}
  ${PLUGINS_CPP_FILES}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)



ament_target_dependencies(${PROJECT_NAME} ${PROJECT_DEPENDENCIES})
pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)

# Add executables
add_executable(gripper_handler_behavior_node ${NODE_SOURCES})
ament_target_dependencies(gripper_handler_behavior_node ${PROJECT_DEPENDENCIES})
target_link_libraries(gripper_handler_behavior_node ${PROJECT_NAME})

# Install executable
install(TARGETS
gripper_handler_behavior_node
  DESTINATION lib/${PROJECT_NAME}
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
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