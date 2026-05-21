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

# pid_controller: prefer system → local clone → FetchContent.
# The local clone lives in plugins/pid_speed_controller/thirdparties/pid_controller/
# so it is fetched only the first time and reused on subsequent builds. The
# thirdparties/ directory carries an AMENT_IGNORE so colcon never indexes it
# as a ROS package and the ament file-walking linters skip the vendored sources.
set(_THIRDPARTIES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparties)
file(MAKE_DIRECTORY ${_THIRDPARTIES_DIR})
file(WRITE ${_THIRDPARTIES_DIR}/AMENT_IGNORE "")

find_package(pid_controller QUIET)
if(${pid_controller_FOUND})
  message(STATUS "pid_controller found system-wide")
else()
  set(LOCAL_PID_SRC ${_THIRDPARTIES_DIR}/pid_controller)
  if(EXISTS ${LOCAL_PID_SRC}/CMakeLists.txt)
    message(STATUS "pid_controller: using local clone at ${LOCAL_PID_SRC}")
    add_subdirectory(${LOCAL_PID_SRC} ${CMAKE_BINARY_DIR}/pid_controller_build)
  else()
    message(STATUS "pid_controller: cloning into ${LOCAL_PID_SRC}")
    include(FetchContent)
    fetchcontent_declare(
      pid_controller
      GIT_REPOSITORY https://github.com/RPS98/pid_controller.git
      GIT_TAG v1.0
      SOURCE_DIR ${LOCAL_PID_SRC}
    )
    fetchcontent_makeavailable(pid_controller)
  endif()
endif()

include_directories(
  include
  include/${PLUGIN_NAME}
  ${EIGEN3_INCLUDE_DIRS}

  # pid_controller headers (vendored under thirdparties/)
  ${_THIRDPARTIES_DIR}/pid_controller/include
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