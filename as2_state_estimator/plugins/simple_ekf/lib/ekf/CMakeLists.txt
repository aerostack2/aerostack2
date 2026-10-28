cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME ekf)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
project(${PROJECT_NAME})

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Set Release as default
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# ament is there when this builds inside the ROS package, and absent when it builds on its own
find_package(ament_cmake QUIET)
if(ament_cmake_FOUND)
  find_package(eigen3_cmake_module REQUIRED)
endif()
find_package(Eigen3 REQUIRED NO_MODULE)

message(STATUS "Using Eigen3: ${EIGEN3_INCLUDE_DIRS}")

set(SOURCE_CPP_FILES
  src/ekf_c_code.cpp
  src/ekf_datatype.cpp
  src/ekf_wrapper.cpp
)

# Shared in the ROS package. Whoever builds it on its own can ask for a static one
if(NOT DEFINED EKF_LIBRARY_TYPE)
  set(EKF_LIBRARY_TYPE SHARED)
endif()
add_library(${PROJECT_NAME} ${EKF_LIBRARY_TYPE} ${SOURCE_CPP_FILES})

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<TARGET_PROPERTY:Eigen3::Eigen,INTERFACE_INCLUDE_DIRECTORIES>
)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

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

if(BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()

if(ament_cmake_FOUND)
  ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
  ament_export_include_directories(include)
  ament_export_dependencies(Eigen3)
  ament_package()
endif()
