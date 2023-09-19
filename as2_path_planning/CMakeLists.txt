cmake_minimum_required(VERSION 3.8)
project(as2_path_planning)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# set Release as default
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_BUILD_TYPE RelWithDebInfo)

set(PROJECT_DEPENDENCIES
  ament_cmake
  ament_cmake_ros
  rclcpp
  rclcpp_action
  nav_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  as2_core
  as2_msgs
  OpenCV
)

find_package(backward_ros REQUIRED)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

include_directories(
  include
  include/${PROJECT_NAME}
)

set(SOURCE_CPP_FILES
  src/laserscan_to_occupancy_grid.cpp
)

add_executable(${PROJECT_NAME}_node src/laserscan_to_occupancy_grid_node.cpp ${SOURCE_CPP_FILES})
ament_target_dependencies(${PROJECT_NAME}_node ${PROJECT_DEPENDENCIES})

add_executable(map_server src/map_server_node.cpp src/map_server.cpp)
ament_target_dependencies(map_server ${PROJECT_DEPENDENCIES})

add_executable(path_planner src/path_planner_node.cpp src/path_planner.cpp)
ament_target_dependencies(path_planner ${PROJECT_DEPENDENCIES})

if(BUILD_TESTING)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)

  ament_cppcheck(src/ include/ tests/)
  ament_clang_format(src/ include/ tests/ --config ${CMAKE_CURRENT_SOURCE_DIR}/.clang-format)
endif()

install(TARGETS
  ${PROJECT_NAME}_node
  map_server
  path_planner
  DESTINATION lib/${PROJECT_NAME})

##################

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
