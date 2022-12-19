cmake_minimum_required(VERSION 3.5)

set(PROJECT_NAME as2_usb_camera_interface)
project(${PROJECT_NAME} VERSION 0.1.0)

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
rclcpp
as2_core
as2_msgs
std_msgs
std_srvs
sensor_msgs
OpenCV
cv_bridge
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()


include_directories(
  include
  include/${PROJECT_NAME}
)

set(HEADER_HPP_FILES 
  include/${PROJECT_NAME}/as2_usb_camera_interface.hpp
)

set(SOURCE_CPP_FILES 
  src/as2_usb_camera_interface.cpp
)

add_executable(${PROJECT_NAME}_node src/as2_usb_camera_interface_node.cpp ${SOURCE_CPP_FILES})
target_link_libraries(${PROJECT_NAME}_node yaml-cpp)
ament_target_dependencies(${PROJECT_NAME}_node ${PROJECT_DEPENDENCIES})


install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)

  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)
  
  ament_cppcheck(src/ include/ tests/)
  ament_clang_format(src/ include/ tests/ --config ${CMAKE_CURRENT_SOURCE_DIR}/.clang-format)

endif()

ament_package()
