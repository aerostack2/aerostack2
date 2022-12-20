cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME as2_pixhawk_platform)
project(${PROJECT_NAME})

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
set(PROJECT_DEPENDENCIES
  ament_cmake
  rclcpp
  as2_core
  as2_msgs
  std_srvs
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2_geometry_msgs
  nav_msgs
  px4_msgs
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

include_directories(
  include
  include/${PROJECT_NAME}

  # uncomment if using Eigen3
  # {EIGEN3_INCLUDE_DIRS}
)

set(SOURCE_CPP_FILES
  src/pixhawk_platform.cpp
)

add_executable(${PROJECT_NAME}_node src/pixhawk_platform_main.cpp ${SOURCE_CPP_FILES})

ament_target_dependencies(${PROJECT_NAME}_node ${PROJECT_DEPENDENCIES})

if(BUILD_TESTING)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)

  ament_cppcheck(src/ include/ tests/)
  ament_clang_format(src/ include/ tests/ --config ${CMAKE_CURRENT_SOURCE_DIR}/.clang-format)

  include(tests/profiling_test.cmake)
  include(tests/unit_test.cmake)
  include(tests/odom_test.cmake)
endif()

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
