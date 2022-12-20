cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME as2_trajectory_generator)
project(${PROJECT_NAME} VERSION 0.2.0)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# set Release as default
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

#set fPIC to ON by default
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

#download the dynamic_trajectory_generator from github
find_package(dynamic_trajectory_generator QUIET)
if (${dynamic_trajectory_generator_FOUND})
  MESSAGE(STATUS "dynamic_trajectory_generator found")
else (${dynamic_trajectory_generator_FOUND})
  MESSAGE(STATUS "dynamic_trajectory_generator not found")
  include(FetchContent)
  FetchContent_Declare(
  dynamic_trajectory_generator
  GIT_REPOSITORY https://github.com/miferco97/dynamic_trajectory_generator.git
  GIT_TAG master
  )
  FetchContent_MakeAvailable(dynamic_trajectory_generator)
endif(${dynamic_trajectory_generator_FOUND})

# add_subdirectory(subpackages/dynamic_trajectory_generator)
# find dependencies
set(PROJECT_DEPENDENCIES
  ament_cmake
  rclcpp
  as2_core
  std_msgs
  as2_msgs
  std_srvs
  as2_behavior
  trajectory_msgs
  geometry_msgs
  visualization_msgs
  yaml-cpp
  as2_motion_reference_handlers
  Eigen3
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

include_directories(
  include
  include/${PROJECT_NAME}
  ${EIGEN3_INCLUDE_DIRS}
)

set(SOURCE_CPP_FILES 
  src/trajectory_generator.cpp
)

add_executable(${PROJECT_NAME}_node src/trajectory_generator_node.cpp ${SOURCE_CPP_FILES})
target_link_libraries(${PROJECT_NAME}_node dynamic_trajectory_generator yaml-cpp)
ament_target_dependencies(${PROJECT_NAME}_node ${PROJECT_DEPENDENCIES})

if(BUILD_TESTING)

  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)
  
  ament_cppcheck(src/ include/ tests/)
  ament_clang_format(src/ include/ tests/ --config ${CMAKE_CURRENT_SOURCE_DIR}/.clang-format)

endif()

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})
  
ament_package()