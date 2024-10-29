cmake_minimum_required(VERSION 3.5)

set(PROJECT_NAME as2_msgs)
project(${PROJECT_NAME} VERSION 1.0.0)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(PROJECT_DEPENDENCIES
  std_msgs
  geometry_msgs
  geographic_msgs
  builtin_interfaces
  nav_msgs
  action_msgs
  sensor_msgs
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()


file(GLOB_RECURSE MSGS_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "msg/*.msg")

file(GLOB_RECURSE SRVS_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "srv/*.srv")

file(GLOB_RECURSE ACTION_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "action/*.action")

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSGS_FILES}
  ${SRVS_FILES}
  ${ACTION_FILES}
  DEPENDENCIES ${PROJECT_DEPENDENCIES}
)

if(BUILD_TESTING)
  find_package(ament_cmake_lint_cmake REQUIRED)
  ament_lint_cmake()
  find_package(ament_cmake_xmllint REQUIRED)
  ament_xmllint()
endif()

ament_package()
