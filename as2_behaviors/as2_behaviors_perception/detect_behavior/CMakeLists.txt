cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME detect_behavior)
project(${PROJECT_NAME} VERSION 1.0.0)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# set Release as default
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Include directories
include_directories(
  include
  include/${PROJECT_NAME}
)

set(SOURCE_CPP_FILES
  src/${PROJECT_NAME}.cpp
  src/${PROJECT_NAME}_node.cpp
)

# find dependencies
set(PROJECT_DEPENDENCIES
  ament_cmake
  ament_cmake_ros
  pluginlib
  rclcpp
  rclcpp_action
  as2_core
  as2_msgs
  as2_behavior
  rclcpp_components
  sensor_msgs
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

# Plugins
# set(PLUGINS_CPP_FILES
#   detect_behavior/
# )


add_executable(${PROJECT_NAME}_node ${SOURCE_CPP_FILES})
# target_link_libraries(${PROJECT_NAME}_node ${OPENCV_LIBS})
ament_target_dependencies(${PROJECT_NAME}_node ${PROJECT_DEPENDENCIES})

# # Library
# add_library(${PROJECT_NAME} ${PLUGINS_CPP_FILES})
# target_include_directories(${PROJECT_NAME} PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)

# ament_target_dependencies(${PROJECT_NAME} ${PROJECT_DEPENDENCIES})
# pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)

install(
  TARGETS ${PROJECT_NAME}_node
  # EXPORT export_${PROJECT_NAME}
  # ARCHIVE DESTINATION lib
  # LIBRARY DESTINATION lib
  # RUNTIME DESTINATION bin
)
# ament_export_libraries(
#   ${PROJECT_NAME}
# )
# ament_export_targets(
#   export_${PROJECT_NAME}
# )

