cmake_minimum_required(VERSION 3.5)

set(PROJECT_NAME as2_aruco_detector)
project(${PROJECT_NAME} VERSION 0.2.0)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# set Release as default
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(PROJECT_DEPENDENCIES
ament_cmake
rclcpp
rclcpp_action
as2_core
as2_behavior
as2_msgs
std_msgs
std_srvs
sensor_msgs
tf2_geometry_msgs
OpenCV
cv_bridge
action_msgs
# builtin_interfaces
# rosidl_default_generators
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()


include_directories(
  include
  include/${PROJECT_NAME}
)

set(SOURCE_CPP_FILES 
  src/as2_aruco_detector.cpp
)

# set(ACTION_FILES
#   action/ArucoDetector.action
# )

# rosidl_generate_interfaces(${PROJECT_NAME}
#   ${ACTION_FILES}
#   DEPENDENCIES builtin_interfaces action_msgs
# )

add_executable(${PROJECT_NAME}_node src/as2_aruco_detector_node.cpp ${SOURCE_CPP_FILES})
target_link_libraries(${PROJECT_NAME}_node ${OPENCV_LIBS})

# # if $ROS_DISTRO is galactic use this else use the other one
# if($ENV{ROS_DISTRO} STREQUAL "galactic")
#   rosidl_target_interfaces(${PROJECT_NAME}_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
# else()
#   rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
#   target_link_libraries(${PROJECT_NAME}_node "${cpp_typesupport_target}")
# endif()

ament_target_dependencies(${PROJECT_NAME}_node ${PROJECT_DEPENDENCIES})

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)

  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)
  
  ament_cppcheck(src/ include/ tests/)
  ament_clang_format(src/ include/ tests/ --config ${CMAKE_CURRENT_SOURCE_DIR}/.clang-format)

endif()

if(BUILD_TESTING)
  #FIXME: add all the other tests
  # include(./tests/CMakeLists.txt)
endif()

ament_package()
