cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME as2_behaviors_motion)
project(${PROJECT_NAME} VERSION 1.0.0)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# set Release as default
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# find dependencies
set(PROJECT_DEPENDENCIES
  ament_cmake
  ament_cmake_ros
  pluginlib
  rclcpp
  rclcpp_action
  as2_core
  as2_msgs
  std_srvs
  as2_behavior
  as2_motion_reference_handlers
  geometry_msgs
  Eigen3
  rclcpp_components
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

# Plugins
set(PLUGINS_CPP_FILES
  follow_path_behavior/plugins/follow_path_plugin_position.cpp
  follow_path_behavior/plugins/follow_path_plugin_trajectory.cpp
  go_to_behavior/plugins/go_to_plugin_position.cpp
  go_to_behavior/plugins/go_to_plugin_trajectory.cpp
  land_behavior/plugins/land_plugin_platform.cpp
  land_behavior/plugins/land_plugin_speed.cpp
  land_behavior/plugins/land_plugin_trajectory.cpp
  takeoff_behavior/plugins/takeoff_plugin_platform.cpp
  takeoff_behavior/plugins/takeoff_plugin_position.cpp
  takeoff_behavior/plugins/takeoff_plugin_speed.cpp
  takeoff_behavior/plugins/takeoff_plugin_trajectory.cpp
)

# Library
add_library(${PROJECT_NAME} ${PLUGINS_CPP_FILES})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME} ${PROJECT_DEPENDENCIES})
pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_targets(
  export_${PROJECT_NAME}
)

# Executable
set(SUBPACKAGES
  follow_path_behavior
  go_to_behavior
  land_behavior
  takeoff_behavior
  follow_reference_behavior
)

foreach(EXECUTABLE ${SUBPACKAGES})
  include_directories(
    ${EXECUTABLE}/include
    ${EXECUTABLE}/include/${PROJECT_NAME}
  )

  add_executable(${EXECUTABLE}_node ${EXECUTABLE}/src/${EXECUTABLE}.cpp ${EXECUTABLE}/src/${EXECUTABLE}_main.cpp)

  target_include_directories(${EXECUTABLE}_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

  ament_target_dependencies(${EXECUTABLE}_node ${PROJECT_DEPENDENCIES})

  install(TARGETS ${EXECUTABLE}_node
    DESTINATION lib/${PROJECT_NAME})

  install(
    DIRECTORY ${EXECUTABLE}/include/
    DESTINATION include
  )

  install(
    DIRECTORY ${EXECUTABLE}/config/
    DESTINATION share/${PROJECT_NAME}/config/${EXECUTABLE}
  )

  ament_export_include_directories(
    include
  )
endforeach()

# Components
add_library(takeoff_component SHARED
  takeoff_behavior/src/takeoff_behavior.cpp
)
ament_target_dependencies(takeoff_component ${PROJECT_DEPENDENCIES})
rclcpp_components_register_nodes(takeoff_component "TakeoffBehavior")

add_library(land_component SHARED
  land_behavior/src/land_behavior.cpp
)
ament_target_dependencies(land_component ${PROJECT_DEPENDENCIES})
rclcpp_components_register_nodes(land_component "LandBehavior")

add_library(go_to_component SHARED
  go_to_behavior/src/go_to_behavior.cpp
)
ament_target_dependencies(go_to_component ${PROJECT_DEPENDENCIES})
rclcpp_components_register_nodes(go_to_component "GoToBehavior")

add_library(follow_path_component SHARED
  follow_path_behavior/src/follow_path_behavior.cpp
)
ament_target_dependencies(follow_path_component ${PROJECT_DEPENDENCIES})
rclcpp_components_register_nodes(follow_path_component "FollowPathBehavior")

add_library(follow_reference_component SHARED
  follow_reference_behavior/src/follow_reference_behavior.cpp
)
ament_target_dependencies(follow_reference_component ${PROJECT_DEPENDENCIES})
rclcpp_components_register_nodes(follow_reference_component "FollowReferenceBehavior")

install(TARGETS
  takeoff_component
  land_component
  go_to_component
  follow_path_component
  follow_reference_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)

  ament_cppcheck(${SUBPACKAGES})

  ament_clang_format(${SUBPACKAGES}
    --config ${CMAKE_CURRENT_SOURCE_DIR}/.clang-format)
endif()

ament_package()
