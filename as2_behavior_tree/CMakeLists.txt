# Set the minimum required CMake version
cmake_minimum_required(VERSION 3.5)

# Set the project name
set(PROJECT_NAME as2_behavior_tree)
project(${PROJECT_NAME})

# Default to C++17 if not set
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Set Release as default build type if not set
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Find dependencies
set(PROJECT_DEPENDENCIES
  ament_cmake
  ament_cmake_ros
  behaviortree_cpp_v3
  nav2_msgs
  rclcpp
  rclcpp_action
  as2_core
  as2_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

# Include necessary directories
include_directories(
  include
  include/${PROJECT_NAME}
)

# Set source files
set(PLUGINS_CPP_FILES
  plugins/action/takeoff_action.cpp
  plugins/action/arm_service.cpp
  plugins/action/offboard_service.cpp
  plugins/action/go_to_action.cpp
  plugins/action/land_action.cpp
  plugins/action/send_event.cpp
  plugins/action/echo.cpp
  plugins/action/set_origin.cpp
  plugins/action/get_origin.cpp
  plugins/action/gps_to_cartesian.cpp
  plugins/action/go_to_gps_action.cpp
  plugins/condition/is_flying_condition.cpp
  plugins/decorator/wait_for_event.cpp
  plugins/decorator/wait_for_alert.cpp
)

# Create the node executable
add_executable(${PROJECT_NAME}_node src/as2_behavior_tree_node.cpp)
ament_target_dependencies(${PROJECT_NAME}_node ${PROJECT_DEPENDENCIES})
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

# Create the library
add_library(${PROJECT_NAME} ${PLUGINS_CPP_FILES})
ament_target_dependencies(${PROJECT_NAME} ${PROJECT_DEPENDENCIES})

# Install the headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install the node executable
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install the library
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install the launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install the resource directory
install(DIRECTORY
  resource
  DESTINATION share/${PROJECT_NAME}
)

# Build tests if testing is enabled
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  add_subdirectory(tests)
  add_subdirectory(tests/node_emulators)
endif()

# Create the ament package
ament_package()
