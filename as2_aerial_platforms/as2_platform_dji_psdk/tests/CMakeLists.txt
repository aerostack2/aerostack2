# GTest

# find dependencies
set(TEST_DEPENDENCIES
ament_cmake_gtest
  ament_index_cpp
)

foreach(DEPENDENCY ${TEST_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

file(GLOB TEST_SOURCE "*_gtest.cpp")

foreach(TEST_FILE ${TEST_SOURCE})
  get_filename_component(TEST_NAME ${TEST_FILE} NAME_WE)

  # Add GTest executable and link necessary dependencies
  ament_add_gtest(${PROJECT_NAME}_${TEST_NAME} ${TEST_FILE})
  ament_target_dependencies(${PROJECT_NAME}_${TEST_NAME} ${PROJECT_DEPENDENCIES} ${TEST_DEPENDENCIES})
  target_link_libraries(${PROJECT_NAME}_${TEST_NAME} ${PROJECT_NAME})
endforeach()