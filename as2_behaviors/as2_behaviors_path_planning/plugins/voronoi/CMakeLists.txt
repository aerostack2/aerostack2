cmake_minimum_required(VERSION 3.5)
set(PLUGIN_NAME voronoi)

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
  geometry_msgs
  nav_msgs
  visualization_msgs
  std_msgs
  nav_msgs
)

foreach(DEPENDENCY ${PLUGIN_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

include_directories(
  include
  include/${PLUGIN_NAME}
  thirdparty
)

set(SOURCE_CPP_FILES
  src/${PLUGIN_NAME}.cpp
  src/${PLUGIN_NAME}_searcher.cpp
  thirdparty/dynamicvoronoi/dynamicvoronoi.cpp
)

# Library
add_library(${PLUGIN_NAME} SHARED ${SOURCE_CPP_FILES})
target_link_libraries(${PLUGIN_NAME} as2_behaviors_path_planning_plugin_base path_planner_common)
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

# PLUGIN TESTS
# if(BUILD_TESTING)
#   add_subdirectory(tests)
# endif()