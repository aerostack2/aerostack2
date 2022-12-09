cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME basic_state_estimator)
project(${PROJECT_NAME} VERSION 0.2.0)

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
  rclcpp
  as2_core
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

include_directories(
  include
  include/${PROJECT_NAME}
)

set(SOURCE_CPP_FILES
  src/basic_state_estimator.cpp
)

# set(INCLUDE_HPP_FILES
# include/${PROJECT_NAME}/as2_node_template_libs.hpp
# )
add_executable(${PROJECT_NAME}_node src/basic_state_estimator_node.cpp ${SOURCE_CPP_FILES})
ament_target_dependencies(${PROJECT_NAME}_node ${PROJECT_DEPENDENCIES})

if(BUILD_TESTING)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)
  
  ament_cppcheck(src/ include/ tests/)
  ament_clang_format(src/ include/ tests/ --config ${CMAKE_CURRENT_SOURCE_DIR}/.clang-format)

endif()

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
