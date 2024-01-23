set(EXECUTABLE_NAME generate_polynomial_trajectory_behavior)

# Download the dynamic_trajectory_generator from github
find_package(dynamic_trajectory_generator QUIET)

if(${dynamic_trajectory_generator_FOUND})
  MESSAGE(STATUS "dynamic_trajectory_generator found")
else(${dynamic_trajectory_generator_FOUND})
  MESSAGE(STATUS "dynamic_trajectory_generator not found")
  include(FetchContent)
  FetchContent_Declare(
    dynamic_trajectory_generator
    GIT_REPOSITORY https://github.com/miferco97/dynamic_trajectory_generator.git
    GIT_TAG master
  )
  FetchContent_MakeAvailable(dynamic_trajectory_generator)
endif(${dynamic_trajectory_generator_FOUND})

# Find dependencies
set(EXECUTABLE_DEPENDENCIES
  trajectory_msgs
  geometry_msgs
  visualization_msgs
  Eigen3
  rclcpp_components
)

foreach(DEPENDENCY ${EXECUTABLE_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

# Include directories
include_directories(
  include
  include/${EXECUTABLE_NAME}
  ${EIGEN3_INCLUDE_DIRS}
)

set(SOURCE_CPP_FILES
  src/${EXECUTABLE_NAME}.cpp
  src/${EXECUTABLE_NAME}_node.cpp
)

add_executable(${EXECUTABLE_NAME}_node ${SOURCE_CPP_FILES})
target_link_libraries(${EXECUTABLE_NAME}_node dynamic_trajectory_generator)
ament_target_dependencies(${EXECUTABLE_NAME}_node ${PROJECT_DEPENDENCIES} ${EXECUTABLE_DEPENDENCIES})

add_library(trajectory_generator_component SHARED
  src/generate_polynomial_trajectory_behavior.cpp
)
target_link_libraries(trajectory_generator_component dynamic_trajectory_generator)
ament_target_dependencies(trajectory_generator_component ${PROJECT_DEPENDENCIES} ${EXECUTABLE_DEPENDENCIES})
rclcpp_components_register_nodes(trajectory_generator_component "DynamicPolynomialTrajectoryGenerator")

install(TARGETS
  trajectory_generator_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

install(TARGETS
  ${EXECUTABLE_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(${EXECUTABLE_DEPENDENCIES})
ament_export_include_directories(include)