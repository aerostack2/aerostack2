cmake_minimum_required(VERSION 3.5)
set(PLUGIN_NAME jerk_limited_trajectory_generator)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Set Release as default
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Plugin dependencies (mostly inherited from the package).
set(PLUGIN_DEPENDENCIES
  ament_cmake
  rclcpp
  pluginlib
  as2_core
  as2_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  Eigen3
)

foreach(DEPENDENCY ${PLUGIN_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

# --- trajectory_generator_jerk_limited: prefer system → local clone → FetchContent
# The local clone lives in plugins/<plugin>/thirdparties/<lib>/ so it is fetched
# only the first time and reused on subsequent builds. The parent thirdparties/
# directory is gitignored repo-wide; we drop an AMENT_IGNORE there at configure
# time so colcon never indexes any nested package.xml files as ROS packages
# and so the ament file-walking linters (cpplint, uncrustify, ...) skip the
# vendored sources.
set(_THIRDPARTIES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparties)
file(MAKE_DIRECTORY ${_THIRDPARTIES_DIR})
file(WRITE ${_THIRDPARTIES_DIR}/AMENT_IGNORE "")

find_package(trajectory_generator_jerk_limited QUIET)
if(${trajectory_generator_jerk_limited_FOUND})
  message(STATUS "trajectory_generator_jerk_limited found system-wide")
else()
  # Skip self-tests of fetched deps (notably glog's cmake_package_config_*),
  # which are sensitive to CMake version and would register against this
  # package's ctest run.
  set(_as2_prev_build_testing ${BUILD_TESTING})
  set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
  set(LOCAL_TGJL_SRC
    ${_THIRDPARTIES_DIR}/trajectory_generator_jerk_limited)
  if(EXISTS ${LOCAL_TGJL_SRC}/CMakeLists.txt)
    message(STATUS
      "trajectory_generator_jerk_limited: using local clone at ${LOCAL_TGJL_SRC}")
    add_subdirectory(${LOCAL_TGJL_SRC}
      ${CMAKE_BINARY_DIR}/trajectory_generator_jerk_limited_build)
  else()
    message(STATUS
      "trajectory_generator_jerk_limited: cloning into ${LOCAL_TGJL_SRC}")
    include(FetchContent)
    fetchcontent_declare(
      trajectory_generator_jerk_limited
      GIT_REPOSITORY https://github.com/aerostack2/jerk_limited_trajectory_generator_lib.git
      GIT_TAG main
      SOURCE_DIR ${LOCAL_TGJL_SRC}
    )
    fetchcontent_makeavailable(trajectory_generator_jerk_limited)
  endif()
  set(BUILD_TESTING ${_as2_prev_build_testing} CACHE BOOL "" FORCE)
  unset(_as2_prev_build_testing)
endif()

include_directories(
  include
  include/${PLUGIN_NAME}
  ${EIGEN3_INCLUDE_DIRS}
)

set(SOURCE_CPP_FILES
  src/${PLUGIN_NAME}.cpp
)

# Plugin shared library (loaded by pluginlib::ClassLoader).
add_library(${PLUGIN_NAME} SHARED ${SOURCE_CPP_FILES})
target_link_libraries(${PLUGIN_NAME}
  ${BEHAVIOR_NAME}_plugin_base
  trajectory_generator_jerk_limited
)
ament_target_dependencies(${PLUGIN_NAME} ${PLUGIN_DEPENDENCIES})

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PLUGIN_NAME})
ament_export_targets(export_${PLUGIN_NAME})

install(
  TARGETS ${PLUGIN_NAME}
  EXPORT export_${PLUGIN_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Re-export upstream target so downstream packages can resolve it.
install(
  TARGETS trajectory_generator_jerk_limited
  EXPORT export_trajectory_generator_jerk_limited
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()
