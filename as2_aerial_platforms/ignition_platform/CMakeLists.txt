cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME ignition_platform)
set(NODE_NAME ignition_platform)
project(${PROJECT_NAME} VERSION 0.2.0)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# set Release as default
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(PROJECT_DEPENDENCIES
  ament_cmake
  ament_cmake_ros
  rclcpp
  as2_core
  as2_msgs
  ignition_assets
  geometry_msgs
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

set(SOURCE_CPP_FILES 
  src/${NODE_NAME}.cpp
)
set(HEADER_HPP_FILES
  include/${NODE_NAME}/${NODE_NAME}.hpp
)

include_directories(
  include
  include/${PROJECT_NAME}
)

add_executable(${PROJECT_NAME}_node src/${NODE_NAME}_main.cpp ${SOURCE_CPP_FILES} ${HEADER_HPP_FILES})
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
