# Set executable name and version
cmake_minimum_required(VERSION 3.8)
set(EXECUTABLE_NAME  mass_estimation_behavior)


# Set executable dependencies
set(EXECUTABLE_DEPENDENCIES
  geometry_msgs
  sensor_msgs
)
# Find dependencies
foreach(DEPENDENCY ${EXECUTABLE_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

# Include directories
include_directories(
  include
  include/${EXECUTABLE_NAME}
)

# Set the source files
set(SOURCE_CPP_FILES
  src/${EXECUTABLE_NAME}.cpp
  src/param_estimation.cpp
  src/${EXECUTABLE_NAME}_node.cpp
)

# Create the node executable
add_executable(${EXECUTABLE_NAME}_node ${SOURCE_CPP_FILES})
target_link_libraries(${EXECUTABLE_NAME}_node)
ament_target_dependencies(${EXECUTABLE_NAME}_node ${PROJECT_DEPENDENCIES} ${EXECUTABLE_DEPENDENCIES})

# Install config
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/${EXECUTABLE_NAME})

# Install launch
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

# Install the node executable
install(TARGETS
  ${EXECUTABLE_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(${EXECUTABLE_DEPENDENCIES})
ament_export_include_directories(include)

add_subdirectory(tests)