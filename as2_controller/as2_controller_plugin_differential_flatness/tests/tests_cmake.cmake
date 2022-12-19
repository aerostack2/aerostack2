find_package(ament_cmake_gtest REQUIRED)

file(GLOB TEST_SOURCES tests/*test.cpp )

# create a test executable for each test file
foreach(TEST_SOURCE ${TEST_SOURCES})

  get_filename_component(_src_filename ${TEST_SOURCE} NAME)
  string(LENGTH ${_src_filename} name_length)
  math(EXPR final_length  "${name_length}-4") # remove .cpp of the name
  string(SUBSTRING ${_src_filename} 0 ${final_length} TEST_NAME)
  
  message(STATUS ${SOURCE_CPP_FILES})
  ament_add_gtest(${TEST_NAME}_test ${TEST_SOURCE} ${SOURCE_CPP_FILES} )
  ament_target_dependencies(${TEST_NAME}_test  ${PROJECT_DEPENDENCIES})
  target_link_libraries(${TEST_NAME}_test gtest_main)

  endforeach()
