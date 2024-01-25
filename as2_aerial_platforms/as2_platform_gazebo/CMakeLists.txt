cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME as2_platform_gazebo)
project(${PROJECT_NAME} VERSION 0.2.2)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# set Release as default
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(PROJECT_DEPENDENCIES
  ament_cmake
  ament_cmake_ros
  rclcpp
  as2_core
  as2_msgs
  as2_gazebo_assets
  geometry_msgs
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

set(SOURCE_CPP_FILES
  src/gazebo_platform.cpp
)

include_directories(
  include
  include/${PROJECT_NAME}
)

add_executable(${PROJECT_NAME}_node src/gazebo_platform_main.cpp ${SOURCE_CPP_FILES})
ament_target_dependencies(${PROJECT_NAME}_node ${PROJECT_DEPENDENCIES})

if(BUILD_TESTING)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)

  ament_cppcheck(src/ include/ tests/)
  ament_clang_format(src/ include/ tests/ --config ${CMAKE_CURRENT_SOURCE_DIR}/.clang-format)
endif()

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
