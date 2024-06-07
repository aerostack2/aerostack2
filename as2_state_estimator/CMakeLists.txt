cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME as2_state_estimator)
project(${PROJECT_NAME} VERSION 1.0.0)

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
  rclcpp
  pluginlib
  as2_core
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  mocap4r2_msgs
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

include_directories(
  include
  include/${PROJECT_NAME}
)

# Plugins
set(PLUGIN_LIST
  raw_odometry
  ground_truth
  mocap_pose
)

# Create a list of all the plugin files
foreach(PLUGINS ${PLUGIN_LIST})
  list(APPEND PLUGINS_CPP_FILES
    plugins/${PLUGINS}/src/${PLUGINS}.cpp
  )

  include_directories(
    plugins/${PLUGINS}/include
    plugins/${PLUGINS}/include/${PLUGINS}
  )
endforeach()

# Library
add_library(${PROJECT_NAME} SHARED ${PLUGINS_CPP_FILES})

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME} ${PROJECT_DEPENDENCIES})
pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_targets(
  export_${PROJECT_NAME}
)

# Executable
add_executable(${PROJECT_NAME}_node src/state_estimator_node.cpp src/state_estimator.cpp)
target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME}_node ${PROJECT_DEPENDENCIES})

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

# For each plugin, install config files
foreach(PLUGINS ${PLUGIN_LIST})
  if(EXISTS plugins/${PLUGINS}/config)
    install(
      DIRECTORY plugins/${PLUGINS}/config
      DESTINATION share/${PROJECT_NAME}/plugins/${PLUGINS}
    )
  endif()
endforeach()

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # For each plugin, check if tests/CMakeLists.txt exists and add it
  foreach(PLUGIN ${PLUGIN_LIST})
    if(EXISTS plugins/${PLUGIN}/tests/CMakeLists.txt)
      add_subdirectory(plugins/${PLUGIN}/tests)
    endif()
  endforeach()
endif()

ament_package()
