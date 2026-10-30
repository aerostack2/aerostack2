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

set(PROJECT_LINK_LIBRARIES
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  as2_behavior::as2_behavior
  ${std_srvs_TARGETS}
  as2_core::as2_core
  ${as2_msgs_TARGETS}
  ${action_msgs_TARGETS}
)

include_directories(
  include
  include/${PROJECT_NAME}
)

# Set source files
set(SOURCE_CPP_FILES
  src/set_arming_state_behavior_main.cpp
  src/set_offboard_mode_behavior_main.cpp
)

add_executable(arm_behavior src/set_arming_state_behavior_main.cpp)
add_executable(offboard_behavior src/set_offboard_mode_behavior_main.cpp)

target_link_libraries(arm_behavior PRIVATE ${PROJECT_LINK_LIBRARIES})
target_link_libraries(offboard_behavior PRIVATE ${PROJECT_LINK_LIBRARIES})

# Create the library
add_library(${PROJECT_NAME} STATIC ${SOURCE_CPP_FILES})
target_link_libraries(${PROJECT_NAME} PUBLIC ${PROJECT_LINK_LIBRARIES})

install(TARGETS
  arm_behavior
  offboard_behavior
  DESTINATION lib/${PROJECT_NAME}
)

# Install the library
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Build tests if testing is enabled
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  file(GLOB_RECURSE EXCLUDE_FILES
    build/*
    install/*
  )
  set(AMENT_LINT_AUTO_FILE_EXCLUDE ${EXCLUDE_FILES})
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(tests)
endif()

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
