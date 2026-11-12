include_directories(
  include
  include/${PROJECT_NAME}
)

add_executable(platform_emulator src/platform_emulator.cpp)
target_include_directories(platform_emulator
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(platform_emulator PRIVATE
  behaviortree_cpp::behaviortree_cpp
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  as2_core::as2_core
  ${as2_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
)

add_executable(takeoff_emulator src/takeoff_emulator.cpp)
target_include_directories(takeoff_emulator
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(takeoff_emulator PRIVATE
  behaviortree_cpp::behaviortree_cpp
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  as2_core::as2_core
  ${as2_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
)

add_executable(land_emulator src/land_emulator.cpp)
target_include_directories(land_emulator
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(land_emulator PRIVATE
  behaviortree_cpp::behaviortree_cpp
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  as2_core::as2_core
  ${as2_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
)

add_executable(go_to_emulator src/go_to_emulator.cpp)
target_include_directories(go_to_emulator
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(go_to_emulator PRIVATE
  behaviortree_cpp::behaviortree_cpp
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  as2_core::as2_core
  ${as2_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
)

add_executable(follow_path_emulator src/follow_path_emulator.cpp)
target_include_directories(follow_path_emulator
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(follow_path_emulator PRIVATE
  behaviortree_cpp::behaviortree_cpp
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  as2_core::as2_core
  ${as2_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

install(TARGETS platform_emulator
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS takeoff_emulator
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS land_emulator
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS go_to_emulator
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS follow_path_emulator
DESTINATION lib/${PROJECT_NAME})
