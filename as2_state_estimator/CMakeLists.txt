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
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)
  ament_cppcheck(src/ include/ tests/ plugins/)
  ament_clang_format(src/ include/ tests/ plugins/ --config ${CMAKE_CURRENT_SOURCE_DIR}/.clang-format)

  # Create a list of all the source files to be tested
  # from /plugins/plugin_name/tests/plugin_test.cpp if file exists
  foreach(PLUGINS ${PLUGIN_LIST})
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/plugins/${PLUGINS}/tests/${PLUGINS}_test.cpp)
      list(APPEND TEST_SOURCE_CPP_FILES
        plugins/${PLUGINS}/tests/${PLUGINS}_test.cpp
      )
    endif()
  endforeach()

  if(TEST_SOURCE_CPP_FILES)
    ament_add_gtest(${PROJECT_NAME}_test ${TEST_SOURCE_CPP_FILES})
    ament_target_dependencies(${PROJECT_NAME}_test ${PROJECT_DEPENDENCIES})
  endif()
endif()

ament_package()
