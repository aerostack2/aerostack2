cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME as2_core VERSION 0.2.2)
project(${PROJECT_NAME})

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

# https://stackoverflow.com/a/73279950/9553849
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};/usr/share/cmake/geographiclib")

# find dependencies
set(PROJECT_DEPENDENCIES
  ament_cmake
  rclcpp
  std_msgs
  nav_msgs
  sensor_msgs
  as2_msgs
  std_srvs
  geometry_msgs
  tf2_geometry_msgs
  tf2_ros
  tf2 
  Eigen3
  rclcpp_lifecycle
  rclcpp_action
  yaml-cpp
  image_transport
  cv_bridge
  geographic_msgs
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

find_package(GeographicLib REQUIRED)

include_directories(
  include
  include/${PROJECT_NAME}
  ${EIGEN3_INCLUDE_DIRS}
)

set(SOURCE_CPP_FILES 
  src/node.cpp
  src/sensor.cpp
  src/aerial_platform.cpp
  src/platform_state_machine.cpp
  src/rate.cpp
  src/core_functions.cpp
  src/utils/control_mode_utils.cpp
  src/utils/tf_utils.cpp
  src/utils/yaml_utils.cpp
  src/utils/frame_utils.cpp
  src/utils/gps_utils.cpp
)

add_library(${PROJECT_NAME} STATIC ${SOURCE_CPP_FILES} )
target_link_libraries(${PROJECT_NAME} yaml-cpp)
target_link_libraries(${PROJECT_NAME} ${GeographicLib_LIBRARIES})
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME} ${PROJECT_DEPENDENCIES})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${PROJECT_DEPENDENCIES})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)
  

  ament_cppcheck(src/ include/ tests/)
  ament_clang_format(src/ include/ tests/ --config ${CMAKE_CURRENT_SOURCE_DIR}/.clang-format)
 
  include(./tests/CMakeLists.txt)

endif()


install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
