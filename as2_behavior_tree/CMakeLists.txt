cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME as2_behavior_tree)
project(${PROJECT_NAME} VERSION 0.2.2)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# set Release as default
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# find dependencies
set(PROJECT_DEPENDENCIES
  ament_cmake
  ament_cmake_ros
  behaviortree_cpp_v3
  nav2_msgs
  rclcpp
  rclcpp_action
  as2_core
  as2_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

include_directories(
  include
  include/${PROJECT_NAME}
)

set(PLUGINS_CPP_FILES
  plugins/action/takeoff_action.cpp
  plugins/action/arm_service.cpp
  plugins/action/offboard_service.cpp
  plugins/action/go_to_action.cpp
  plugins/action/land_action.cpp
  plugins/action/send_event.cpp
  plugins/action/echo.cpp
  plugins/action/set_origin.cpp
  plugins/action/get_origin.cpp
  plugins/action/gps_to_cartesian.cpp
  plugins/action/go_to_gps_action.cpp
  plugins/condition/is_flying_condition.cpp
  plugins/decorator/wait_for_event.cpp
  plugins/decorator/wait_for_alert.cpp
)

add_library(${PROJECT_NAME} ${PLUGINS_CPP_FILES})
target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  ${PROJECT_NAME}
  ${PROJECT_DEPENDENCIES}
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${PROJECT_DEPENDENCIES})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

add_executable(${PROJECT_NAME}_main src/as2_behavior_tree_node.cpp)
target_include_directories(${PROJECT_NAME}_main
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}_main ${PROJECT_DEPENDENCIES})
target_link_libraries(${PROJECT_NAME}_main ${PROJECT_NAME})

# ## TEST ###
add_subdirectory(test)

# ###########

# ## NODE EMULATORS ###
add_subdirectory(test/node_emulators)

# #####################
install(DIRECTORY
  resource
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}_main
  DESTINATION lib/${PROJECT_NAME})

ament_package()
