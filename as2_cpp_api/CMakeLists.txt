cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME as2_cpp_api)
project(${PROJECT_NAME} VERSION 1.0.0)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
set(PROJECT_DEPENDENCIES
  ament_cmake
  rclcpp
  rclcpp_action
  std_srvs
  as2_msgs
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

# Set source files
set(SOURCE_CPP_FILES
  # src/behavior_actions/behavior_handler.cpp
  src/behavior_actions/take_off_behavior.cpp
  src/modules/takeoff_module.cpp
  src/behavior_actions/go_to_behavior.cpp
  src/modules/go_to_module.cpp
  src/behavior_actions/land_behavior.cpp
  src/modules/land_module.cpp
  src/service_clients/service_handler.cpp
  src/service_clients/arming.cpp
  src/service_clients/offboard.cpp
  src/drone_interface_base.cpp
  src/drone_interface.cpp
)

# Create the library
add_library(${PROJECT_NAME} STATIC ${SOURCE_CPP_FILES})
ament_target_dependencies(${PROJECT_NAME} ${PROJECT_DEPENDENCIES})

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC $<INSTALL_INTERFACE:include>
         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         ${CMAKE_CURRENT_SOURCE_DIR}/third_party_libraries)

add_executable(mission examples/mission.cpp)
ament_target_dependencies(mission ${PROJECT_DEPENDENCIES})
target_link_libraries(mission ${PROJECT_NAME})
target_include_directories(
  mission
  PUBLIC $<INSTALL_INTERFACE:include>
         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

install(TARGETS
  mission
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

ament_package()

