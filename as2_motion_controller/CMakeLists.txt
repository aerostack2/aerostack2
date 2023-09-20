cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME as2_motion_controller)
project(${PROJECT_NAME} VERSION 1.0.0)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# set Release as default
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# set fPIC to ON by default
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# find dependencies
set(PROJECT_DEPENDENCIES
  ament_cmake
  ament_cmake_ros
  rclcpp
  rclcpp_action
  pluginlib
  std_srvs
  as2_core
  as2_msgs
  as2_motion_reference_handlers
  geometry_msgs
  Eigen3
  benchmark
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

if(${pid_controller_FOUND})
  MESSAGE(STATUS "pid_controller found")
else(${pid_controller_FOUND})
  MESSAGE(STATUS "pid_controller not found")
  include(FetchContent)
  FetchContent_Declare(
    pid_controller
    GIT_REPOSITORY https://github.com/RPS98/pid_controller.git
    GIT_TAG main
  )
  FetchContent_MakeAvailable(pid_controller)
endif(${pid_controller_FOUND})

include_directories(
  include
  include/${PROJECT_NAME}
  ${EIGEN3_INCLUDE_DIRS}

  # Include pid_controller
  ${pid_controller_INCLUDE_DIRS}
)

# Plugins
set(PLUGIN_LIST
  differential_flatness_controller
  pid_speed_controller
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

# Link with pid_controller
target_link_libraries(${PROJECT_NAME} pid_controller)

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
add_executable(${PROJECT_NAME}_node src/controller_manager_node.cpp src/controller_manager.cpp src/controller_handler.cpp)
target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME}_node ${PROJECT_DEPENDENCIES})

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

# For each plugin, install config files
foreach(PLUGINS ${PLUGIN_LIST})
  install(
    DIRECTORY plugins/${PLUGINS}/config
    DESTINATION share/${PROJECT_NAME}/plugins/${PLUGINS}
  )
endforeach()

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)

  ament_cppcheck(src/ include/ tests/ plugins/)
  ament_clang_format(src/ include/ tests/ plugins/ --config ${CMAKE_CURRENT_SOURCE_DIR}/.clang-format)

  include(tests/unit_test.cmake)
endif()

ament_package()