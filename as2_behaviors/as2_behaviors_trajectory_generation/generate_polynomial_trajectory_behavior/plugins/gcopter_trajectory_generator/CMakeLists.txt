cmake_minimum_required(VERSION 3.5)
set(PLUGIN_NAME gcopter_trajectory_generator)

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

# --- Sanitize upstream BUILD_* options inherited via cache.
# When the user (or an IDE like VSCode CMake Tools) passes
# -DBUILD_PYBIND=ON we honour it as long as the required tools are present
# on the host. Otherwise we downgrade the option to OFF (with a warning) so
# the configure step still succeeds. Keeps the wrapper plugin tolerant to
# heterogeneous environments without touching package.xml dependencies.
if(BUILD_PYBIND)
  find_package(Python3 COMPONENTS Interpreter Development QUIET)
  find_package(pybind11 QUIET)
  if(NOT Python3_FOUND OR NOT pybind11_FOUND)
    message(WARNING
      "[gcopter_trajectory_generator] BUILD_PYBIND=ON but Python3 "
      "Development or pybind11 not found; disabling BUILD_PYBIND for "
      "upstream gcopter_lib. Install python3-dev and pybind11-dev to "
      "re-enable.")
    set(BUILD_PYBIND OFF CACHE BOOL "" FORCE)
  endif()
endif()

# --- gcopter_lib: prefer system → local clone → FetchContent
# The local clone lives in plugins/<plugin>/thirdparties/<lib>/ so it is fetched
# only the first time and reused on subsequent builds. The parent thirdparties/
# directory is gitignored repo-wide; we drop an AMENT_IGNORE there at configure
# time so colcon never indexes the upstream package.xml files as ROS packages
# and so the ament file-walking linters (cpplint, uncrustify, ...) skip the
# vendored sources.
set(_THIRDPARTIES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparties)
file(MAKE_DIRECTORY ${_THIRDPARTIES_DIR})
file(WRITE ${_THIRDPARTIES_DIR}/AMENT_IGNORE "")

find_package(gcopter_lib QUIET)
if(${gcopter_lib_FOUND})
  message(STATUS "gcopter_lib found system-wide")
else()
  # Skip self-tests of fetched deps (notably glog's cmake_package_config_*),
  # which are sensitive to CMake version and would register against this
  # package's ctest run.
  set(_as2_prev_build_testing ${BUILD_TESTING})
  set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
  set(LOCAL_GCOPTER_SRC ${_THIRDPARTIES_DIR}/gcopter_trajectory_generator_lib)
  if(EXISTS ${LOCAL_GCOPTER_SRC}/CMakeLists.txt)
    message(STATUS "gcopter_trajectory_generator_lib: using local clone at ${LOCAL_GCOPTER_SRC}")
    add_subdirectory(${LOCAL_GCOPTER_SRC} ${CMAKE_BINARY_DIR}/gcopter_trajectory_generator_lib_build)
  else()
    message(STATUS "gcopter_trajectory_generator_lib: cloning into ${LOCAL_GCOPTER_SRC}")
    include(FetchContent)
    fetchcontent_declare(
      gcopter_trajectory_generator_lib
      GIT_REPOSITORY https://github.com/aerostack2/gcopter_trajectory_generator_lib.git
      GIT_TAG main
      SOURCE_DIR ${LOCAL_GCOPTER_SRC}
    )
    fetchcontent_makeavailable(gcopter_trajectory_generator_lib)
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
  gcopter_lib
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
  TARGETS gcopter_lib
  EXPORT export_gcopter_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()
