cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME detect_behavior)
project(${PROJECT_NAME} VERSION 1.0.0)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# set Release as default
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Set commons dependencies
set(PROJECT_DEPENDENCIES
  ament_cmake
  ament_cmake_ros
  rclcpp
  pluginlib
  rclcpp_action
  as2_core
  as2_msgs
  std_msgs
  as2_behavior
  geometry_msgs
  builtin_interfaces
  rosidl_default_generators
  rclcpp_components
  sensor_msgs
)

# Find dependencies
foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

include_directories(
  include
  include/${PROJECT_NAME}
)

# Set files base name
set(FILES_NAME detect_behavior)

# Create plugin base library
set(PLUGIN_BASE_FILES
  include/${PROJECT_NAME}/${FILES_NAME}_plugin_base.hpp
)

# Create plugin base library
add_library(${PROJECT_NAME}_plugin_base SHARED ${PLUGIN_BASE_FILES})
# Specify the language for the target as C++ (as it is a header only library)
set_target_properties(${PROJECT_NAME}_plugin_base PROPERTIES LINKER_LANGUAGE CXX)
ament_target_dependencies(${PROJECT_NAME}_plugin_base ${PROJECT_DEPENDENCIES})

# Create as2_behaviors_gripper library
set(SOURCE_CPP_FILES
  src/${FILES_NAME}.cpp
)

add_library(${PROJECT_NAME}_lib SHARED ${SOURCE_CPP_FILES})
ament_target_dependencies(${PROJECT_NAME}_lib ${PROJECT_DEPENDENCIES})

# Executables
set(SOURCE_CPP_FILE
  src/${FILES_NAME}_node.cpp
)

# Add executables
add_executable(${PROJECT_NAME}_node ${SOURCE_CPP_FILES})
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_lib)
ament_target_dependencies(${PROJECT_NAME}_node ${PROJECT_DEPENDENCIES})

target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


# rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(${PROJECT_NAME}_node ${cpp_typesupport_target})
target_link_libraries(${PROJECT_NAME}_lib ${cpp_typesupport_target})

# Export libraries and targets
install(
  TARGETS
    ${PROJECT_NAME}_lib
    ${PROJECT_NAME}_plugin_base
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# Export headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Add Plugins
# set(PLUGIN_LIST
#   two_fingers
#   dc_servo
# )

# foreach(PLUGIN ${PLUGIN_LIST})
#   add_subdirectory(plugins/${PLUGIN})
# endforeach()

# pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)

# # For each plugin, install its config file
# foreach(PLUGIN ${PLUGIN_LIST})
#   install(
#     DIRECTORY plugins/${PLUGIN}/config
#     DESTINATION share/${PROJECT_NAME}/plugins/${PLUGIN}
#   )
# endforeach()

# Install launch and config files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # add_subdirectory(tests)
endif()

ament_package()