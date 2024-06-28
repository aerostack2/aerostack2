cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME as2_geozones)
project(${PROJECT_NAME} VERSION 0.2.0)

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
  sensor_msgs
  nav_msgs
  geometry_msgs
  geographic_msgs
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

# Include necessary directories
include_directories(
  include
  include/${PROJECT_NAME}
)

# Set source files
set(SOURCE_CPP_FILES
  src/as2_geozones.cpp
)

# Create the node executable
add_executable(${PROJECT_NAME}_node src/as2_geozones_node.cpp ${SOURCE_CPP_FILES})
ament_target_dependencies(${PROJECT_NAME}_node ${PROJECT_DEPENDENCIES})

# Create the library
add_library(${PROJECT_NAME} STATIC ${SOURCE_CPP_FILES})
ament_target_dependencies(${PROJECT_NAME} ${PROJECT_DEPENDENCIES})

# Target interface with executable and library
ament_export_dependencies(rosidl_default_runtime)

# Install the headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install the config and launch directory
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME})

# Install the node executable
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

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
    include/${PROJECT_NAME}/pnpoly.hpp
    build/*
    install/*
  )
  set(AMENT_LINT_AUTO_FILE_EXCLUDE ${EXCLUDE_FILES})
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(tests)
endif()

ament_package()