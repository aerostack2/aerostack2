cmake_minimum_required(VERSION 3.8)
set(PROJECT_NAME  as2_behaviors_param_estimation)
project(${PROJECT_NAME} VERSION 1.0.0)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
set(CMAKE_CXX_STANDARD 17)
endif()

# Set Release as default
if(NOT CMAKE_BUILD_TYPE)
set(CMAKE_BUILD_TYPE Release)
endif()

# Set commons dependencies
set(PROJECT_DEPENDENCIES
  ament_cmake
  rclcpp
  rclcpp_action
  as2_core
  as2_msgs
  as2_behavior
  geometry_msgs
  Eigen3
  backward_ros
  sensor_msgs
)
# Find dependencies
foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

# Add param_estimation as a dependency
# find_package(param_estimation QUIET)
if(${param_estimation_FOUND})
  message(STATUS "param_estimation found")
else()
  message(STATUS "param_estimation not found")
  include(FetchContent)
  fetchcontent_declare(
    param_estimation
    GIT_REPOSITORY https://github.com/carmendrpr/param_estimation.git 
    # GIT_TAG main
    GIT_TAG new_organization
    )
  fetchcontent_makeavailable(param_estimation)
endif()


# Include directories
include_directories(
  include
  include/${PROJECT_NAME}
  ${EIGEN3_INCLUDE_DIRS}
  ${CMAKE_BINARY_DIR}/_deps/param_estimation-src/include
)
set(SOURCE_CPP_FILES
  src/param_estimation_behavior.cpp
)
# Create the node executable
add_executable(param_estimation_behavior_node src/param_estimation_behavior_node.cpp ${SOURCE_CPP_FILES})
ament_target_dependencies(param_estimation_behavior_node ${PROJECT_DEPENDENCIES})
target_link_libraries(param_estimation_behavior_node param_estimation)


# Create the library
add_library(${PROJECT_NAME} STATIC ${SOURCE_CPP_FILES} )
ament_target_dependencies(${PROJECT_NAME} ${PROJECT_DEPENDENCIES})
target_link_libraries(${PROJECT_NAME} param_estimation)

# Install the headers
install(
  DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME})
# Install the node executable
install(TARGETS
  param_estimation_behavior_node
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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  file(GLOB_RECURSE EXCLUDE_FILES
    build/*
    install/*
  )
  set(AMENT_LINT_AUTO_FILE_EXCLUDE ${EXCLUDE_FILES})
  ament_lint_auto_find_test_dependencies()
endif()


ament_package()