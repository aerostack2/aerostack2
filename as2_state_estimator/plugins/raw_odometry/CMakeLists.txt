cmake_minimum_required(VERSION 3.5)
set(PLUGIN_NAME raw_odometry)

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
  rclcpp
  pluginlib
  as2_core
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
)

foreach(DEPENDENCY ${PLUGIN_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

include_directories(
  include
  include/${PLUGIN_NAME}
)

set(SOURCE_CPP_FILES
  src/${PLUGIN_NAME}.cpp
)

# Library
add_library(${PLUGIN_NAME} SHARED ${SOURCE_CPP_FILES})
target_link_libraries(${PLUGIN_NAME} ${PROJECT_NAME} ${PROJECT_NAME}_plugin_base)
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
