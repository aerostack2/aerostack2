find_package(ament_cmake_gtest REQUIRED)

file(GLOB TEST_SOURCES *.cpp)

# create a test executable for each test file
foreach(TEST_SOURCE ${TEST_SOURCES})

  get_filename_component(_src_filename ${TEST_SOURCE} NAME)
  string(LENGTH ${_src_filename} name_length)
  math(EXPR final_length "${name_length}-4") # remove .cpp of the name
  string(SUBSTRING ${_src_filename} 0 ${final_length} TEST_NAME)

  ament_add_gtest(${TEST_NAME} ${TEST_SOURCE})
  ament_target_dependencies(${TEST_NAME} ${PROJECT_DEPENDENCIES})
  target_link_libraries(${TEST_NAME} gtest_main ${PROJECT_NAME} yaml-cpp)
endforeach()
