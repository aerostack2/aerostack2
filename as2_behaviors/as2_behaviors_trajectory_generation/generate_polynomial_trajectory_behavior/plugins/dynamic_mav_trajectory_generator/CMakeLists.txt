cmake_minimum_required(VERSION 3.5)
set(PLUGIN_NAME dynamic_mav_trajectory_generator)

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

# --- dynamic_trajectory_generator: prefer system → local clone → FetchContent
# The local clone lives in plugins/<plugin>/thirdparties/<lib>/ so it is fetched
# only the first time and reused on subsequent builds. The parent thirdparties/
# directory is gitignored repo-wide; we drop an AMENT_IGNORE there at configure
# time so colcon never indexes the upstream package.xml files as ROS packages
# and so the ament file-walking linters (cpplint, uncrustify, ...) skip the
# vendored sources.
set(_THIRDPARTIES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparties)
file(MAKE_DIRECTORY ${_THIRDPARTIES_DIR})
file(WRITE ${_THIRDPARTIES_DIR}/AMENT_IGNORE "")

find_package(dynamic_trajectory_generator QUIET)
if(${dynamic_trajectory_generator_FOUND})
  message(STATUS "dynamic_trajectory_generator found system-wide")
else()
  set(LOCAL_DTG_SRC ${_THIRDPARTIES_DIR}/dynamic_trajectory_generator)
  if(EXISTS ${LOCAL_DTG_SRC}/CMakeLists.txt)
    message(STATUS "dynamic_trajectory_generator: using local clone at ${LOCAL_DTG_SRC}")
    add_subdirectory(${LOCAL_DTG_SRC} ${CMAKE_BINARY_DIR}/dynamic_trajectory_generator_build)
  else()
    message(STATUS "dynamic_trajectory_generator: cloning into ${LOCAL_DTG_SRC}")
    include(FetchContent)
    fetchcontent_declare(
      dynamic_trajectory_generator
      GIT_REPOSITORY https://github.com/cvar-upm/dynamic_trajectory_generator.git
      GIT_TAG v1.1
      SOURCE_DIR ${LOCAL_DTG_SRC}
    )
    fetchcontent_makeavailable(dynamic_trajectory_generator)
  endif()
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
  dynamic_trajectory_generator
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

# Re-export upstream targets so downstream packages can resolve them.
install(
  TARGETS dynamic_trajectory_generator
  EXPORT export_dynamic_trajectory_generator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  TARGETS mav_trajectory_generation
  EXPORT export_mav_trajectory_generation
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()
