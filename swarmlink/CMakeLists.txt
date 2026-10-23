cmake_minimum_required(VERSION 3.8)
project(swarmlink)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set( DEPENDENCIES
  as2_msgs
  rclcpp
)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
foreach(pkg ${DEPENDENCIES})
  find_package(${pkg} REQUIRED)
endforeach()


include_directories(
  include
  include/${PROJECT_NAME}
)

add_library(${PROJECT_NAME} SHARED src/swarmlink.cpp src/swarmlink_client.cpp)
ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES})

add_executable(swarmlink_node src/swarmlink_node.cpp)
target_link_libraries(swarmlink_node ${PROJECT_NAME})
ament_target_dependencies(swarmlink_node ${DEPENDENCIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${DEPENDENCIES})

install(TARGETS swarmlink_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  add_subdirectory(tests)
endif()

ament_package()
