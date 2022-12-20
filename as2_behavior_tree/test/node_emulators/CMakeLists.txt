cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME node_emulators)
project(${PROJECT_NAME} VERSION 0.2.0)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# set Release as default
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# find dependencies
set(PROJECT_DEPENDENCIES
  ament_cmake
  ament_cmake_ros

  rclcpp
  rclcpp_action
  as2_core
  as2_msgs
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

include_directories(
  include
  include/${PROJECT_NAME}
)

add_executable(platform_emulator src/platform_emulator.cpp)
target_include_directories(platform_emulator
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(platform_emulator ${PROJECT_DEPENDENCIES})

add_executable(takeoff_emulator src/takeoff_emulator.cpp)
target_include_directories(takeoff_emulator
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(takeoff_emulator ${PROJECT_DEPENDENCIES})

add_executable(land_emulator src/land_emulator.cpp)
target_include_directories(land_emulator
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(land_emulator ${PROJECT_DEPENDENCIES})

add_executable(goto_emulator src/goto_emulator.cpp)
target_include_directories(goto_emulator
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(goto_emulator ${PROJECT_DEPENDENCIES})

add_executable(follow_path_emulator src/follow_path_emulator.cpp)
target_include_directories(follow_path_emulator
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(follow_path_emulator ${PROJECT_DEPENDENCIES})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

install(TARGETS platform_emulator
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS takeoff_emulator
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS land_emulator
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS goto_emulator
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS follow_path_emulator
DESTINATION lib/${PROJECT_NAME})

ament_package()
