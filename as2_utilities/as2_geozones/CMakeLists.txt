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

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED)

# find dependencies
set(PROJECT_DEPENDENCIES
  ament_cmake
  rclcpp
  as2_core
  as2_msgs
  sensor_msgs
  builtin_interfaces
  nav_msgs
  geometry_msgs
  geographic_msgs
)

set(MESSAGE_DEPENDENCIES
  as2_msgs
  std_msgs
  geometry_msgs
  builtin_interfaces
)

set(SERVICE_DEPENDENCIES
  builtin_interfaces
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

set(SRVS_FILES
"srv/GetGeozone.srv"
"srv/SetGeozone.srv"
)

set(MSGS_FILES
"msg/Geozone.msg"
"msg/Polygonlist.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSGS_FILES}
  ${SRVS_FILES}
  DEPENDENCIES ${MESSAGE_DEPENDENCIES}
)

include_directories(
  include
  include/${PROJECT_NAME}
)

set(SOURCE_CPP_FILES
  src/as2_geozones.cpp
)

# set(INCLUDE_HPP_FILES
# include/${PROJECT_NAME}/as2_node_template_libs.hpp
# )
add_executable(${PROJECT_NAME}_node src/as2_geozones_node.cpp ${SOURCE_CPP_FILES})
ament_target_dependencies(${PROJECT_NAME}_node ${PROJECT_DEPENDENCIES})

add_library(${PROJECT_NAME}_lib SHARED tests/as2_geozones_gtest.cpp ${SOURCE_CPP_FILES})
ament_target_dependencies(${PROJECT_NAME}_lib ${PROJECT_DEPENDENCIES})

rosidl_target_interfaces(${PROJECT_NAME}_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

rosidl_target_interfaces(${PROJECT_NAME}_lib
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(${PROJECT_NAME}_node nlohmann_json::nlohmann_json)
target_link_libraries(${PROJECT_NAME}_lib nlohmann_json::nlohmann_json)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}_lib)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}_lib
  DESTINATION lib/${PROJECT_NAME}_lib)

# find_program(CMAKE_CXX_CPPCHECK NAMES cppcheck)
# if(CMAKE_CXX_CPPCHECK)
#     list(APPEND CMAKE_CXX_CPPCHECK
#       "--enable=all"
#       "--inconclusive"
#       "--force"
#       "-i/include/geozones/pnpoly.hpp" # Add the path to the file you want to exclude
#     )
# endif()

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

