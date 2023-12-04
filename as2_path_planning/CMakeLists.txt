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
  as2_motion_reference_handlers
  OpenCV
  grid_map_msgs
  grid_map_ros
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

# Creating library for test linking
add_library(${PROJECT_NAME} STATIC ${SOURCE_CPP_FILES} )
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME} ${PROJECT_DEPENDENCIES})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${PROJECT_DEPENDENCIES})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
###

add_executable(${PROJECT_NAME}_node src/laserscan_to_occupancy_grid_node.cpp src/laserscan_to_occupancy_grid.cpp)
ament_target_dependencies(${PROJECT_NAME}_node ${PROJECT_DEPENDENCIES})

add_executable(map_server src/map_server_node.cpp src/map_server.cpp)
ament_target_dependencies(map_server ${PROJECT_DEPENDENCIES})

add_executable(path_planner src/path_planner_node.cpp src/path_planner.cpp)
ament_target_dependencies(path_planner ${PROJECT_DEPENDENCIES})

add_executable(explorer src/explorer_node.cpp src/explorer.cpp)
ament_target_dependencies(explorer ${PROJECT_DEPENDENCIES})

add_executable(bug_algorithm src/bug_algorithm_node.cpp src/bug_algorithm.cpp)
ament_target_dependencies(bug_algorithm ${PROJECT_DEPENDENCIES})

add_executable(frontier_allocator src/frontier_allocator_node.cpp src/frontier_allocator.cpp)
ament_target_dependencies(frontier_allocator ${PROJECT_DEPENDENCIES})

add_executable(safeguard src/safeguard_node.cpp src/safeguard.cpp src/drone_watcher.cpp)
ament_target_dependencies(safeguard ${PROJECT_DEPENDENCIES})

if(BUILD_TESTING)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)

  ament_cppcheck(src/ include/ tests/)
  ament_clang_format(src/ include/ tests/ --config ${CMAKE_CURRENT_SOURCE_DIR}/.clang-format)

  include(./tests/CMakeLists.txt)
endif()

install(TARGETS
  ${PROJECT_NAME}_node
  map_server
  path_planner
  explorer
  bug_algorithm
  frontier_allocator
  safeguard
  DESTINATION lib/${PROJECT_NAME})

##################

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
