cmake_minimum_required(VERSION 3.5)
set(PLUGIN_NAME pid_speed_controller)

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

# find dependencies
set(PLUGIN_DEPENDENCIES
  ament_cmake
  rclcpp
  pluginlib
  as2_core
  as2_msgs
  as2_motion_reference_handlers
  geometry_msgs
  Eigen3
)

foreach(DEPENDENCY ${PLUGIN_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

if(${pid_controller_FOUND})
  message(STATUS "pid_controller found")
else()
  message(STATUS "pid_controller not found")
  include(FetchContent)
  fetchcontent_declare(
    pid_controller
    GIT_REPOSITORY https://github.com/RPS98/pid_controller.git
    GIT_TAG v1.0
  )
  fetchcontent_makeavailable(pid_controller)
endif()

include_directories(
  include
  include/${PLUGIN_NAME}
  ${EIGEN3_INCLUDE_DIRS}

  # Incluye pid_controller
  ${CMAKE_BINARY_DIR}/_deps/pid_controller-src/include
)

set(SOURCE_CPP_FILES
  src/${PLUGIN_NAME}.cpp
)

# Library
add_library(${PLUGIN_NAME} SHARED ${SOURCE_CPP_FILES})
target_link_libraries(${PLUGIN_NAME} ${PROJECT_NAME} ${PROJECT_NAME}_plugin_base pid_controller)
ament_target_dependencies(${PLUGIN_NAME} ${PLUGIN_DEPENDENCIES})

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PLUGIN_NAME}
)
ament_export_targets(
  export_${PLUGIN_NAME}
)

install(
  TARGETS ${PLUGIN_NAME}
  EXPORT export_${PLUGIN_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()