cmake_minimum_required(VERSION 3.5)
set(PLUGIN_NAME detector_yolo_gates)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(PLUGIN_DEPENDENCIES
  ament_cmake
  rclcpp
  pluginlib
  std_msgs
  as2_core
  as2_msgs
  as2_behavior
  geometry_msgs
  OpenCV
  sensor_msgs
)

foreach(DEPENDENCY ${PLUGIN_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

set(SOURCE_CPP_FILES
  src/${PLUGIN_NAME}.cpp
)

add_library(${PLUGIN_NAME} SHARED ${SOURCE_CPP_FILES})

target_include_directories(${PLUGIN_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PLUGIN_NAME}
  ${PROJECT_NAME}_lib
  object_perception_plugin_base
)

ament_target_dependencies(${PLUGIN_NAME} ${PLUGIN_DEPENDENCIES})

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config
  DESTINATION share/${PLUGIN_NAME}
)

install(
  TARGETS ${PLUGIN_NAME}
  EXPORT export_${PLUGIN_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(${PLUGIN_NAME})
ament_export_targets(export_${PLUGIN_NAME})

if(BUILD_TESTING)
  # add_subdirectory(tests)
endif()