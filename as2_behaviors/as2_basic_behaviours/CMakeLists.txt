cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME as2_basic_behaviours)
project(${PROJECT_NAME} VERSION 0.2.0)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
set(PROJECT_DEPENDENCIES
  ament_cmake
  rclcpp
  rclcpp_action
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

set(SOURCE_CPP_FILES 
)

set(HEADER_HPP_FILES
)

include_directories(
)

if(BUILD_TESTING)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)
  
  ament_cppcheck(src/ include/)
  ament_clang_format(src/ include/)

endif()

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
