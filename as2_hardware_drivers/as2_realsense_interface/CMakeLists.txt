# Set the minimum required CMake version
cmake_minimum_required(VERSION 3.5)

# Set the project name
set(PROJECT_NAME as2_realsense_interface)
project(${PROJECT_NAME})

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
  as2_core
  as2_msgs
  std_msgs
  std_srvs
  sensor_msgs
  nav_msgs
  geometry_msgs
  realsense2
  tf2
  tf2_ros
)
foreach(dependency ${PROJECT_DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach()

# Include necessary directories
include_directories(
  include
  include/${PROJECT_NAME}
  ${realsense2_INCLUDE_DIR}
)

# Set source files
set(SOURCE_CPP_FILES
  src/as2_realsense_interface.cpp
)

# Create the node executable
add_executable(${PROJECT_NAME}_node src/as2_realsense_interface_node.cpp ${SOURCE_CPP_FILES})
ament_target_dependencies(${PROJECT_NAME}_node ${PROJECT_DEPENDENCIES})
target_link_libraries(${PROJECT_NAME}_node ${realsense2_LIBRARY})

# Create the library
add_library(${PROJECT_NAME} STATIC ${SOURCE_CPP_FILES})
ament_target_dependencies(${PROJECT_NAME} ${PROJECT_DEPENDENCIES})
target_link_libraries(${PROJECT_NAME} ${realsense2_LIBRARY})

# Install the headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install the node executable
install(TARGETS
  ${PROJECT_NAME}_node
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

# Install the launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install the config directory
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
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

# Create the ament package
ament_package()
