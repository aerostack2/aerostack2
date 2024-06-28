set(EXECUTABLE_NAME path_planner_behavior)

# Find dependencies
set(EXECUTABLE_DEPENDENCIES
  ament_cmake
  ament_cmake_ros
  pluginlib
  as2_core
  as2_msgs
  as2_behavior
  # rclcpp_components
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2_ros
)

foreach(DEPENDENCY ${EXECUTABLE_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

# Include directories
include_directories(
  include
  include/${EXECUTABLE_NAME}
)

set(SOURCE_CPP_FILES
  src/${EXECUTABLE_NAME}.cpp
  src/${EXECUTABLE_NAME}_node.cpp
)

# Library for plugins
add_library(${EXECUTABLE_NAME} SHARED plugins/a_star.cpp)
target_include_directories(${EXECUTABLE_NAME} PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
ament_target_dependencies(${EXECUTABLE_NAME} ${EXECUTABLE_DEPENDENCIES})

# Node executable
add_executable(${EXECUTABLE_NAME}_node ${SOURCE_CPP_FILES})
target_include_directories(${EXECUTABLE_NAME}_node PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  ${EXECUTABLE_NAME}_node
  ${PROJECT_DEPENDENCIES}
  ${EXECUTABLE_DEPENDENCIES}
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Library for composing the behavior
add_library(path_planner_component SHARED
  src/path_planner_behavior.cpp
)
ament_target_dependencies(path_planner_component ${PROJECT_DEPENDENCIES} ${EXECUTABLE_DEPENDENCIES})
rclcpp_components_register_nodes(path_planner_component "PathPlannerBehavior")

install(
  TARGETS
    ${EXECUTABLE_NAME}
    path_planner_component
  EXPORT export_${EXECUTABLE_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${EXECUTABLE_NAME}
)
ament_export_targets(
  export_${EXECUTABLE_NAME}
)

pluginlib_export_plugin_description_file(
  ${EXECUTABLE_NAME} 
  plugins.xml
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

install(TARGETS
  ${EXECUTABLE_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(${EXECUTABLE_DEPENDENCIES})
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  file(GLOB_RECURSE EXCLUDE_FILES
    build/*
    install/*
  )
  set(AMENT_LINT_AUTO_FILE_EXCLUDE ${EXCLUDE_FILES})
  ament_lint_auto_find_test_dependencies()
endif()