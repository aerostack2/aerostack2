cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME controller_plugin_differential_flatness)
project(${PROJECT_NAME} VERSION 0.2.0)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# set Release as default
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

#set fPIC to ON by default
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
#opposite to fPIC is fPIE



# find dependencies
set(PROJECT_DEPENDENCIES
  ament_cmake
  rclcpp
  as2_core
  as2_msgs
  geometry_msgs
  trajectory_msgs
  nav_msgs
  Eigen3
  pluginlib
  controller_plugin_base
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

include_directories(
  include
  include/${PROJECT_NAME}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED src/DF_controller_plugin.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  ${PROJECT_NAME}
  ${PROJECT_DEPENDENCIES}
)

if(BUILD_TESTING)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)
  
  ament_cppcheck(src/ include/ tests/)
  ament_clang_format(src/ include/ tests/ --config ${CMAKE_CURRENT_SOURCE_DIR}/.clang-format)

  # include(tests/profiling_cmake.cmake)
  # include(tests/tests_cmake.cmake)
endif()

pluginlib_export_plugin_description_file(controller_plugin_base plugins.xml)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
