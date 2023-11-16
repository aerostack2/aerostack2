cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME differential_flatness_controller)
project(${PROJECT_NAME} VERSION 1.0.0)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# set Release as default
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# set fPIC to ON by default
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# find dependencies
set(PROJECT_DEPENDENCIES
  ament_cmake
  rclcpp
  pluginlib
  as2_core
  as2_msgs
  as2_motion_reference_handlers
  geometry_msgs
  Eigen3
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

include_directories(
  include
  include/${PROJECT_NAME}
  ${EIGEN3_INCLUDE_DIRS}
)

set(SOURCE_CPP_FILES
  src/${PROJECT_NAME}.cpp
)

# Library
add_library(${PROJECT_NAME} SHARED ${SOURCE_CPP_FILES})
target_link_libraries(${PROJECT_NAME} as2_motion_controller_plugin_base)
ament_target_dependencies(${PROJECT_NAME} ${PROJECT_DEPENDENCIES})

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_targets(
  export_${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  set(TEST_DEPENDENCIES
    benchmark
    ament_cmake_gtest
  )

  foreach(DEPENDENCY ${TEST_DEPENDENCIES})
    find_package(${DEPENDENCY} REQUIRED)
  endforeach()

  # Tests
  file(GLOB TEST_SOURCE "tests/*_test.cpp")

  foreach(TEST_FILE ${TEST_SOURCE})
    get_filename_component(TEST_NAME ${TEST_FILE} NAME_WE)

    add_executable(${PROJECT_NAME}_${TEST_NAME} ${TEST_FILE})
    target_link_libraries(${PROJECT_NAME}_${TEST_NAME} ${PROJECT_NAME})
  endforeach()

  # GTest
  file(GLOB TEST_SOURCE "tests/*_gtest.cpp")

  foreach(TEST_FILE ${TEST_SOURCE})
    get_filename_component(TEST_NAME ${TEST_FILE} NAME_WE)

    ament_add_gtest(${PROJECT_NAME}_${TEST_NAME} ${TEST_FILE})
    target_link_libraries(${PROJECT_NAME}_${TEST_NAME} ${PROJECT_NAME})
  endforeach()

  # Benchmark
  file(GLOB TEST_SOURCE "tests/*_benchmark.cpp")

  foreach(TEST_FILE ${TEST_SOURCE})
    get_filename_component(TEST_NAME ${TEST_FILE} NAME_WE)

    add_executable(${PROJECT_NAME}_${TEST_NAME} ${TEST_FILE})
    target_link_libraries(${PROJECT_NAME}_${TEST_NAME} benchmark::benchmark ${PROJECT_NAME})
  endforeach()
endif()
