
find_package(gtest QUIET)
if (${Gtest_FOUND})
  MESSAGE(STATUS "Found Gtest.")
else (${Gtest_FOUND})
  MESSAGE(STATUS "Could not locate Gtest.")
  include(FetchContent)
  FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/609281088cfefc76f9d0ce82e1ff6c30cc3591e5.zip
  )
  # For Windows: Prevent overriding the parent project's compiler/linker settings
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)

endif(${Gtest_FOUND})

include(GoogleTest)

enable_testing()
# find all *.cpp files in the tests directory

file(GLOB TEST_SOURCES tests/*test.cpp )

# create a test executable for each test file
foreach(TEST_SOURCE ${TEST_SOURCES})

  get_filename_component(_src_filename ${TEST_SOURCE} NAME)
  string(LENGTH ${_src_filename} name_length)
  math(EXPR final_length  "${name_length}-4") # remove .cpp of the name
  string(SUBSTRING ${_src_filename} 0 ${final_length} TEST_NAME)
  
  add_executable(${TEST_NAME}_test ${TEST_SOURCE} src/controller_manager.cpp src/controller_handler.cpp)
  ament_target_dependencies(${TEST_NAME}_test  ${PROJECT_DEPENDENCIES})
  target_link_libraries(${TEST_NAME}_test gtest_main)

  # add the test executable to the list of executables to build
  gtest_discover_tests(${TEST_NAME}_test)

  endforeach()

