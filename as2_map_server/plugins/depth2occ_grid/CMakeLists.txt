cmake_minimum_required(VERSION 3.5)
set(PLUGIN_NAME depth2occ_grid)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# set Release as default
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# find dependencies
set(PLUGIN_DEPENDENCIES
  ament_cmake
  depthimage_to_laserscan
)

foreach(DEPENDENCY ${PLUGIN_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

# PLUGIN TESTS
# if(BUILD_TESTING)
#   add_subdirectory(tests)
# endif()