cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME as2_behaviors_platform)
project(${PROJECT_NAME} VERSION 1.0.0)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
set(PROJECT_DEPENDENCIES
  ament_cmake
  rclcpp
  rclcpp_action
  as2_behavior
  std_srvs
  as2_core
  as2_msgs
  action_msgs
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

include_directories(
  include
  include/${PROJECT_NAME}
)

add_executable(arm_behavior src/set_arming_state_behavior_main.cpp)
add_executable(offboard_behavior src/set_offboard_mode_behavior_main.cpp)

ament_target_dependencies(arm_behavior ${PROJECT_DEPENDENCIES})
ament_target_dependencies(offboard_behavior ${PROJECT_DEPENDENCIES})

install(TARGETS
  arm_behavior
  offboard_behavior
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)

  ament_cppcheck(src/ include/)
  ament_clang_format(src/ include/ tests/ --config ${CMAKE_CURRENT_SOURCE_DIR}/.clang-format)
endif()

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
