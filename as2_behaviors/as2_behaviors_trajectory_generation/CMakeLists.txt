cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME as2_behaviors_trajectory_generation)
project(${PROJECT_NAME} VERSION 1.0.0)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Set Release as default
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Set fPIC to ON by default
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Download the dynamic_trajectory_generator from github
find_package(dynamic_trajectory_generator QUIET)

if(${dynamic_trajectory_generator_FOUND})
  MESSAGE(STATUS "dynamic_trajectory_generator found")
else(${dynamic_trajectory_generator_FOUND})
  MESSAGE(STATUS "dynamic_trajectory_generator not found")
  include(FetchContent)
  FetchContent_Declare(
    dynamic_trajectory_generator
    GIT_REPOSITORY https://github.com/miferco97/dynamic_trajectory_generator.git
    GIT_TAG master
  )
  FetchContent_MakeAvailable(dynamic_trajectory_generator)
endif(${dynamic_trajectory_generator_FOUND})

# Set commons dependencies
set(PROJECT_DEPENDENCIES
  ament_cmake
  rclcpp
  std_msgs
  std_srvs
  as2_core
  as2_msgs
  as2_behavior
  as2_motion_reference_handlers
  trajectory_msgs
  geometry_msgs
  visualization_msgs
  Eigen3
  rclcpp_components
)

# Find dependencies
foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

set(SUBPACKAGES
  generate_polynomial_trajectory_behavior
)

foreach(EXECUTABLE ${SUBPACKAGES})
  # Include directories
  include_directories(
    ${EXECUTABLE}/include
    ${EXECUTABLE}/include/${EXECUTABLE}
    ${EIGEN3_INCLUDE_DIRS}
  )

  add_executable(${EXECUTABLE}_node ${EXECUTABLE}/src/${EXECUTABLE}.cpp ${EXECUTABLE}/src/${EXECUTABLE}_node.cpp)
  target_link_libraries(${EXECUTABLE}_node dynamic_trajectory_generator)
  ament_target_dependencies(${EXECUTABLE}_node ${PROJECT_DEPENDENCIES})

  install(TARGETS
    ${EXECUTABLE}_node
    DESTINATION lib/${PROJECT_NAME}
  )

  # Component
  add_library(${EXECUTABLE}_component SHARED
    ${EXECUTABLE}/src/${EXECUTABLE}.cpp
  )
  target_link_libraries(${EXECUTABLE}_component dynamic_trajectory_generator)
  ament_target_dependencies(${EXECUTABLE}_component
    ${PROJECT_DEPENDENCIES}
  )
  rclcpp_components_register_nodes(${EXECUTABLE}_component "DynamicPolynomialTrajectoryGenerator")

  install(TARGETS
    ${EXECUTABLE}_component
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

  install(DIRECTORY
  ${EXECUTABLE}/launch
    DESTINATION share/${PROJECT_NAME}
  )

  install(
    DIRECTORY ${EXECUTABLE}/include/
    DESTINATION include
  )

  ament_export_include_directories(include)
endforeach()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)

  ament_cppcheck(${BEHAVIORS_LIST})
  ament_clang_format(${BEHAVIORS_LIST} --config ${CMAKE_CURRENT_SOURCE_DIR}/.clang-format)
endif()

ament_export_dependencies(${PROJECT_DEPENDENCIES})

ament_package()
