cmake_minimum_required(VERSION 3.16)
set(PROJECT_NAME simple_ekf_core)
project(${PROJECT_NAME})

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Set Release as default
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Inside the ROS package, the plugin adds this directory after the ekf library, and both are
# shared. Built on its own (plain CMake, or `pip install` for the Python bindings), it adds the
# ekf library itself, and both are static and position independent, so that the Python module
# carries everything it needs.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  set(CMAKE_POSITION_INDEPENDENT_CODE ON)
  set(EKF_LIBRARY_TYPE STATIC)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../ekf ${CMAKE_CURRENT_BINARY_DIR}/ekf)
  set(SIMPLE_EKF_CORE_LIBRARY_TYPE STATIC)
else()
  set(SIMPLE_EKF_CORE_LIBRARY_TYPE SHARED)
endif()

# On by default when built by `pip install`: scikit-build-core sets SKBUILD, to a version string
# rather than to ON
if(SKBUILD)
  set(SIMPLE_EKF_CORE_BUILD_PYTHON_DEFAULT ON)
else()
  set(SIMPLE_EKF_CORE_BUILD_PYTHON_DEFAULT OFF)
endif()
option(SIMPLE_EKF_CORE_BUILD_PYTHON "Build the Python bindings"
  ${SIMPLE_EKF_CORE_BUILD_PYTHON_DEFAULT})

# Neither rclcpp nor any message package: tf2 is here only for its header-only LinearMath,
# which ROS installs under /opt/ros/<distro>/include/tf2
find_package(Eigen3 REQUIRED NO_MODULE)
file(GLOB ROS_INCLUDE_DIRS /opt/ros/*/include)
find_path(TF2_INCLUDE_DIR tf2/LinearMath/Transform.h
  PATH_SUFFIXES tf2
  HINTS ${ROS_INCLUDE_DIRS})
if(NOT TF2_INCLUDE_DIR)
  message(FATAL_ERROR
    "tf2's LinearMath headers were not found. Install ros-<distro>-tf2, or pass "
    "-DTF2_INCLUDE_DIR=<the directory that contains tf2/LinearMath>")
endif()

set(SOURCE_CPP_FILES
  src/filter.cpp
  src/logging.cpp
)

add_library(${PROJECT_NAME} ${SIMPLE_EKF_CORE_LIBRARY_TYPE} ${SOURCE_CPP_FILES})

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../ekf/include>
  $<INSTALL_INTERFACE:include>
  $<TARGET_PROPERTY:Eigen3::Eigen,INTERFACE_INCLUDE_DIRECTORIES>
  ${TF2_INCLUDE_DIR}
)
target_link_libraries(${PROJECT_NAME} ekf Eigen3::Eigen)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(SIMPLE_EKF_CORE_BUILD_PYTHON)
  add_subdirectory(python)
endif()
