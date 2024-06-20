cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME as2_external_object_to_tf)
project(${PROJECT_NAME})
set(as2_external_object_to_tf_FOUND 1)
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# set Release as default
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# find dependencies
set(PROJECT_DEPENDENCIES
  ament_cmake
  rclcpp
  as2_core
  as2_msgs
  std_srvs
  std_msgs
  geometry_msgs
  yaml-cpp
  sensor_msgs
  builtin_interfaces
  tf2
  tf2_ros
  mocap4r2_msgs
)

set(SERVICE_DEPENDENCIES
  as2_msgs
  std_srvs
  std_msgs
  geometry_msgs
  sensor_msgs
  builtin_interfaces
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

set(SRVS_FILES
"srv/AddStaticTransform.srv"
"srv/AddStaticTransformGps.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
${SRVS_FILES}
DEPENDENCIES ${SERVICE_DEPENDENCIES}
)

# rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

include_directories(
  include
  include/${PROJECT_NAME}
  #uncomment if using Eigen3
  #{EIGEN3_INCLUDE_DIRS}
)

set(SOURCE_CPP_FILES
  src/as2_external_object_to_tf.cpp
)

add_executable(${PROJECT_NAME}_node src/as2_external_object_to_tf_node.cpp ${SOURCE_CPP_FILES})
ament_target_dependencies(${PROJECT_NAME}_node ${PROJECT_DEPENDENCIES})

add_library(${PROJECT_NAME}_lib SHARED tests/as2_external_object_to_tf_gtest.cpp ${SOURCE_CPP_FILES})
ament_target_dependencies(${PROJECT_NAME}_lib ${PROJECT_DEPENDENCIES})

rosidl_target_interfaces(${PROJECT_NAME}_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

rosidl_target_interfaces(${PROJECT_NAME}_lib
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(${PROJECT_NAME}_node yaml-cpp)
# target_link_libraries(${PROJECT_NAME}_node "${cpp_typesupport_target}")

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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(tests)
endif()

ament_package()
