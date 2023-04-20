cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME as2_motion_reference_handlers) 
project(${PROJECT_NAME} VERSION 1.0.0)

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

# find dependencies
set(PROJECT_DEPENDENCIES
  ament_cmake
  rclcpp
  rclpy
  as2_core
  as2_msgs
  std_srvs
  std_msgs
  rclcpp_action
  geometry_msgs
  trajectory_msgs
  Eigen3
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

include_directories(
  include
  include/${PROJECT_NAME}
  ${EIGEN3_INCLUDE_DIRS}
)

set(SOURCE_CPP_FILES 
  src/basic_motion_references.cpp
  src/hover_motion.cpp
  src/trajectory_motion.cpp
  src/position_motion.cpp
  src/speed_motion.cpp
  src/speed_in_a_plane_motion.cpp
  )

add_library(${PROJECT_NAME} ${SOURCE_CPP_FILES})
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME} ${PROJECT_DEPENDENCIES})
ament_export_include_directories(include)    
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ament_cmake
  ${PROJECT_DEPENDENCIES}
)
ament_export_libraries(${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)
  
  ament_cppcheck(src/ include/ tests/)
  ament_clang_format(src/ include/ tests/ --config ${CMAKE_CURRENT_SOURCE_DIR}/.clang-format)
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

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

ament_package()
