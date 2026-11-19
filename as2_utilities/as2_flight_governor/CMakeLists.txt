cmake_minimum_required(VERSION 3.8)
project(as2_flight_governor)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(yaml-cpp REQUIRED)

set(DEPENDENCIES
  rclcpp
  as2_core
  as2_msgs
  geometry_msgs
  std_msgs
  std_srvs
)
foreach(pkg ${DEPENDENCIES})
  find_package(${pkg} REQUIRED)
endforeach()

# ROS-independent core: governor, policies and YAML loader
add_library(${PROJECT_NAME}_core
  src/governor_core.cpp
  src/policies.cpp
  src/policy_loader.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_core yaml-cpp)

# ROS 2 node
add_library(${PROJECT_NAME} src/flight_governor.cpp)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_core)
ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES})

add_executable(${PROJECT_NAME}_node src/flight_governor_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}_core ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPENDENCIES} yaml-cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(tests)
endif()

ament_package()
