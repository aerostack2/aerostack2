cmake_minimum_required(VERSION 3.5)
set(PLUGIN_NAME scan2occ_grid)

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
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  OpenCV
)

foreach(DEPENDENCY ${PLUGIN_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

include_directories(
  include
  include/${PLUGIN_NAME}
)

add_library(${PLUGIN_NAME} SHARED src/${PLUGIN_NAME}.cpp)
target_link_libraries(${PLUGIN_NAME} PUBLIC
  as2_map_server_base
  rclcpp::rclcpp
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${OpenCV_LIBS}
)

install(
  DIRECTORY include/
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

# PLUGIN TESTS
# if(BUILD_TESTING)
#   add_subdirectory(tests)
# endif()