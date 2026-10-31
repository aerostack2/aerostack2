set(SUBPROJECT_NAME  gripper_behavior)

# Set commons dependencies (inherited from parent + specific ones)
set(SUBPROJECT_DEPENDENCIES
  ${PROJECT_DEPENDENCIES}
)

# Find dependencies
foreach(DEPENDENCY ${SUBPROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

# Include directories
include_directories(
  include
  include/${SUBPROJECT_NAME}
)

# Create plugin base library
set(PLUGIN_BASE_FILES
  include/${SUBPROJECT_NAME}/${SUBPROJECT_NAME}_plugin_base.hpp
)

add_library(${SUBPROJECT_NAME}_plugin_base SHARED ${PLUGIN_BASE_FILES})
set_target_properties(${SUBPROJECT_NAME}_plugin_base PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(${SUBPROJECT_NAME}_plugin_base PUBLIC
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  pluginlib::pluginlib
  as2_core::as2_core
  as2_behavior::as2_behavior
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${as2_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
)

# Create main behavior library
set(BEHAVIOR_SOURCE_FILES
  src/${SUBPROJECT_NAME}.cpp
)

add_library(${SUBPROJECT_NAME}_lib SHARED ${BEHAVIOR_SOURCE_FILES})
target_link_libraries(${SUBPROJECT_NAME}_lib PUBLIC
  ${SUBPROJECT_NAME}_plugin_base
)

# Create executable
set(NODE_SOURCE_FILES
  src/as2_${SUBPROJECT_NAME}_node.cpp
)

add_executable(as2_${SUBPROJECT_NAME}_node ${NODE_SOURCE_FILES})
target_link_libraries(as2_${SUBPROJECT_NAME}_node
  ${SUBPROJECT_NAME}_lib
)

target_include_directories(as2_${SUBPROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Export libraries and targets
install(
  TARGETS
    ${SUBPROJECT_NAME}_lib
    ${SUBPROJECT_NAME}_plugin_base
  EXPORT export_${SUBPROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  TARGETS as2_${SUBPROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# Export headers
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}/${SUBPROJECT_NAME}
)
# Add Plugins
set(PLUGIN_LIST
  two_fingers
  dc_servo
)

foreach(PLUGIN ${PLUGIN_LIST})
  add_subdirectory(plugins/${PLUGIN})
endforeach()

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/${SUBPROJECT_NAME}/launch
)

# Install plugin configs
foreach(PLUGIN ${PLUGIN_LIST})
  install(
    DIRECTORY plugins/${PLUGIN}/config/
    DESTINATION share/${PROJECT_NAME}/${SUBPROJECT_NAME}/plugins/${PLUGIN}/config
  )
endforeach()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests)
    add_subdirectory(tests)
  endif()
endif()

ament_export_dependencies(${SUBPROJECT_DEPENDENCIES})
ament_export_targets(export_${SUBPROJECT_NAME})