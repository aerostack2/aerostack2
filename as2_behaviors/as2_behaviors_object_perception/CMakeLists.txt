cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME as2_behaviors_object_perception)
project(${PROJECT_NAME} VERSION 1.0.0)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(PROJECT_DEPENDENCIES
  ament_cmake
  ament_cmake_ros
  ament_index_cpp
  rclcpp
  pluginlib
  rclcpp_action
  as2_core
  as2_msgs
  std_msgs
  as2_behavior
  geometry_msgs
  builtin_interfaces
  rosidl_default_generators
  rclcpp_components
  sensor_msgs
  cv_bridge
  OpenCV
  yolo_inference_cpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  yaml_cpp_vendor
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

include_directories(
  include
  include/${PROJECT_NAME}
)

set(FILES_NAME as2_behaviors_object_perception)

set(PLUGIN_BASE_FILES
  include/${PROJECT_NAME}/detection_plugin_base.hpp
)

add_library(detection_plugin_base SHARED ${PLUGIN_BASE_FILES})
set_target_properties(detection_plugin_base PROPERTIES LINKER_LANGUAGE CXX)
ament_target_dependencies(detection_plugin_base ${PROJECT_DEPENDENCIES})

target_include_directories(detection_plugin_base PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

set(IMAGE_PREPROCESSING_FILES
  src/common/img_preprocessing.cpp
)

add_library(img_preprocessing SHARED ${IMAGE_PREPROCESSING_FILES})
ament_target_dependencies(img_preprocessing ${PROJECT_DEPENDENCIES})

target_include_directories(img_preprocessing PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

set(SOURCE_CPP_FILES
  src/${FILES_NAME}.cpp
)

add_library(${PROJECT_NAME}_lib SHARED ${SOURCE_CPP_FILES})
ament_target_dependencies(${PROJECT_NAME}_lib ${PROJECT_DEPENDENCIES})
target_link_libraries(${PROJECT_NAME}_lib
  detection_plugin_base
  img_preprocessing
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

set(SOURCE_CPP_FILE
  src/${FILES_NAME}_node.cpp
)

add_executable(${PROJECT_NAME}_node ${SOURCE_CPP_FILE})
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_lib)
ament_target_dependencies(${PROJECT_NAME}_node ${PROJECT_DEPENDENCIES})

target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

set(PLUGIN_LIST
  detector_yolo_gates
  tracker_tf
)

foreach(PLUGIN ${PLUGIN_LIST})

  add_library(${PROJECT_NAME}_${PLUGIN} SHARED
    plugins/${PLUGIN}/src/${PLUGIN}.cpp
  )

  ament_target_dependencies(${PROJECT_NAME}_${PLUGIN}
    ${PROJECT_DEPENDENCIES}
  )

  target_link_libraries(${PROJECT_NAME}_${PLUGIN}
    detection_plugin_base
  )

  target_include_directories(${PROJECT_NAME}_${PLUGIN} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/plugins/${PLUGIN}/include>
    $<INSTALL_INTERFACE:include>
  )

  install(
    TARGETS ${PROJECT_NAME}_${PLUGIN}
    EXPORT export_${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
  )

  install(
    DIRECTORY plugins/${PLUGIN}/include/
    DESTINATION include
    OPTIONAL
  )

endforeach()

install(
  TARGETS
    ${PROJECT_NAME}_lib
    detection_plugin_base
    img_preprocessing
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY plugins/tracker_tf/config
  DESTINATION share/${PROJECT_NAME}/plugins/tracker_tf
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
