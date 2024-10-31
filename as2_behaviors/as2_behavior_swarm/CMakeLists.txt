cmake_minimum_required(VERSION 3.8)
set(PROJECT_NAME  as2_behavior_swarm)
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
  rclcpp_action
  as2_core
  as2_msgs
  as2_behavior
  geometry_msgs
  as2_behavior_swarm_msgs
  as2_motion_reference_handlers
  tf2
  tf2_ros
)
set (SUBPROJECT_NAME
  swarm_behavior_node
  )
find_package(backward_ros REQUIRED)

# Find dependencies
foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

# Include directories
include_directories(
  include
  include/${PROJECT_NAME}
  include/$(SUBPROJECT_NAME)
)


# Create the library
add_library(${PROJECT_NAME} SHARED src/swarm_behavior.cpp src/drone_swarm.cpp src/swarm_utils.cpp)

# Include headers in the library
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME} ${PROJECT_DEPENDENCIES})

add_executable(swarm_behavior_node src/swarm_behavior_node.cpp)
ament_target_dependencies(swarm_behavior_node ${PROJECT_DEPENDENCIES})
target_link_libraries(swarm_behavior_node  ${PROJECT_NAME})


# Install the library
install(
  DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

install(TARGETS
  ${SUBPROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_export_targets(
  export_${PROJECT_NAME}
)


install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  file(GLOB_RECURSE EXCLUDE_FILES
    build/*
    install/*
  )
  set(AMENT_LINT_AUTO_FILE_EXCLUDE ${EXCLUDE_FILES})
  ament_lint_auto_find_test_dependencies()
  # add_subdirectory(tests)
endif()


ament_package()