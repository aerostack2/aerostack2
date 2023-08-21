cmake_minimum_required(VERSION 3.5)

set(PROJECT_NAME as2_realsense_interface)
project(${PROJECT_NAME} VERSION 1.0.0)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(PROJECT_DEPENDENCIES
  ament_cmake
  rclcpp
  as2_core
  as2_msgs
  std_msgs
  std_srvs
  sensor_msgs
  nav_msgs
  geometry_msgs
  realsense2
  tf2
  tf2_ros
  )

foreach(dependency ${PROJECT_DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach()

include_directories(
  include
  include/${PROJECT_NAME}
  ${realsense2_INCLUDE_DIR}
)

set(SOURCE_CPP_FILES 
  src/as2_realsense_interface.cpp
)

add_executable(${PROJECT_NAME}_node src/as2_realsense_interface_node.cpp ${SOURCE_CPP_FILES})
target_link_libraries(${PROJECT_NAME}_node ${realsense2_LIBRARY})
ament_target_dependencies(${PROJECT_NAME}_node ${PROJECT_DEPENDENCIES})

if(BUILD_TESTING)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)
  
  ament_cppcheck(src/ include/ tests/)
  ament_clang_format(src/ include/ tests/ --config ${CMAKE_CURRENT_SOURCE_DIR}/.clang-format)

endif()

install(DIRECTORY
  launch
  config
  DESTINATION 
  share/${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION 
  lib/${PROJECT_NAME})

ament_package()
