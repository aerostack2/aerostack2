set(EXECUTABLE_NAME generate_polynomial_trajectory_behavior)

# Download the dynamic_trajectory_generator from github
find_package(dynamic_trajectory_generator QUIET)

if(${dynamic_trajectory_generator_FOUND})
  message(STATUS "dynamic_trajectory_generator found")
else()
  message(STATUS "dynamic_trajectory_generator not found")
  include(FetchContent)
  fetchcontent_declare(
    dynamic_trajectory_generator
    GIT_REPOSITORY https://github.com/miferco97/dynamic_trajectory_generator.git
    GIT_TAG master
  )
  fetchcontent_makeavailable(dynamic_trajectory_generator)
endif()

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

# Install libraries
install(TARGETS
  trajectory_generator_component
  EXPORT export_trajectory_generator_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  dynamic_trajectory_generator
  EXPORT export_dynamic_trajectory_generator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  mav_trajectory_generation
  EXPORT export_mav_trajectory_generation
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

# Install executables
install(TARGETS
  ${EXECUTABLE_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(${EXECUTABLE_DEPENDENCIES})
ament_export_include_directories(include)

# Build tests if testing is enabled
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  file(GLOB_RECURSE EXCLUDE_FILES
    build/*
    install/*
  )
  set(AMENT_LINT_AUTO_FILE_EXCLUDE ${EXCLUDE_FILES})
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(tests)
endif()