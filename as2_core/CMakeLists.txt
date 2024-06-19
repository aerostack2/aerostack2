cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME as2_core VERSION 0.2.2)
project(${PROJECT_NAME})

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# set Release as default
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# set fPIC to ON by default
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# add -Wall -Wextra -pedantic
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic")

# https://stackoverflow.com/a/73279950/9553849
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};/usr/share/cmake/geographiclib")

# set a define with the system env variable ROS_DISTRO
set(ROS_DISTRO $ENV{ROS_DISTRO})
message(STATUS "Detected ROS_DISTRO: ${ROS_DISTRO}")
if(ROS_DISTRO STREQUAL "galactic")
  option(ROS_GALACTIC "ROS Galactic" ON)
endif()
configure_file(
  "${PROJECT_SOURCE_DIR}/include/as2_core/custom/tf2_geometry_msgs.hpp.in"
  "${PROJECT_SOURCE_DIR}/include/as2_core/custom/tf2_geometry_msgs.hpp"
)

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
    geographic_msgs)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

find_package(GeographicLib REQUIRED)

# Pybind11
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)

ament_python_install_package(${PROJECT_NAME})

include_directories(include include/${PROJECT_NAME} ${EIGEN3_INCLUDE_DIRS})

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
    src/utils/gps_utils.cpp)

# Create the library
add_library(${PROJECT_NAME} STATIC ${SOURCE_CPP_FILES})
target_link_libraries(${PROJECT_NAME} yaml-cpp)
target_link_libraries(${PROJECT_NAME} ${GeographicLib_LIBRARIES})

# Mocks library
set(MOCKS_LIBRARY_NAME mocks)
include(tests/mocks/CMakeLists.txt)

target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME} ${PROJECT_DEPENDENCIES})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${PROJECT_DEPENDENCIES})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

# Python bindings
pybind11_add_module(as2_names src/_as2_core_pybind11.cpp)
target_link_libraries(as2_names PRIVATE
  rclcpp::rclcpp
)

install(
  TARGETS as2_names
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  add_subdirectory(tests)

endif()

install(DIRECTORY include/ DESTINATION include)

install(
  TARGETS ${PROJECT_NAME} ${MOCKS_LIBRARY_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)

ament_package()
