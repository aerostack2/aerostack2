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
  rclcpp
  pluginlib
  rclcpp_action
  as2_core
  as2_msgs
  std_msgs
  as2_behavior
  sensor_msgs
  geometry_msgs
  cv_bridge
  OpenCV
  tf2
  as2_usb_camera_interface
  visualization_msgs
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

set(PROJECT_LINK_LIBRARIES
  rclcpp::rclcpp
  pluginlib::pluginlib
  rclcpp_action::rclcpp_action
  as2_core::as2_core
  ${as2_msgs_TARGETS}
  ${std_msgs_TARGETS}
  as2_behavior::as2_behavior
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  cv_bridge::cv_bridge
  ${OpenCV_LIBS}
  tf2::tf2
  as2_usb_camera_interface::as2_usb_camera_interface
  ${visualization_msgs_TARGETS}
)

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
target_link_libraries(detection_plugin_base PUBLIC ${PROJECT_LINK_LIBRARIES})

target_include_directories(detection_plugin_base PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

set(IMAGE_PREPROCESSING_FILES
  src/common/img_preprocessing.cpp
)

add_library(img_preprocessing SHARED ${IMAGE_PREPROCESSING_FILES})
target_link_libraries(img_preprocessing PUBLIC ${PROJECT_LINK_LIBRARIES})

target_include_directories(img_preprocessing PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

set(SOURCE_CPP_FILES
  src/${FILES_NAME}.cpp
)

add_library(${PROJECT_NAME}_lib SHARED ${SOURCE_CPP_FILES})
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

target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

set(PLUGIN_LIST
  aruco
)

foreach(PLUGIN ${PLUGIN_LIST})

  add_library(${PROJECT_NAME}_${PLUGIN} SHARED
    plugins/${PLUGIN}/src/${PLUGIN}.cpp
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

  install(
    DIRECTORY plugins/${PLUGIN}/config
    DESTINATION share/${PROJECT_NAME}/plugins/${PLUGIN}
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

ament_export_include_directories(include)
ament_export_libraries(detection_plugin_base img_preprocessing)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  pluginlib
  as2_core
  as2_msgs
  as2_behavior
  sensor_msgs
  geometry_msgs
  std_msgs
  OpenCV
  tf2
  visualization_msgs
  as2_usb_camera_interface
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
