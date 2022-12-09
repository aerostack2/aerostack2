find_package(benchmark QUIET)
if (${benckmark_FOUND})
  MESSAGE(STATUS "Found Google Benchmark.")
else (${benckmark_FOUND})
  MESSAGE(STATUS "Could not find Google Benchmark.")
  include(FetchContent)
  FetchContent_Declare(
  benchmark
  URL https://github.com/google/benchmark/archive/fe2e8aa1b4b01a8d2a7675c1edb3fb0ed48ce11c.zip
  )
  set(BENCHMARK_ENABLE_TESTING FALSE) 
  FetchContent_MakeAvailable(benchmark)


endif(${benckmark_FOUND})

include(GoogleTest)

enable_testing()
# find all *.cpp files in the tests directory

file(GLOB TEST_SOURCES tests/*benchmark.cpp )

# create a test executable for each test file
foreach(TEST_SOURCE ${TEST_SOURCES})

  get_filename_component(_src_filename ${TEST_SOURCE} NAME)
  string(LENGTH ${_src_filename} name_length)
  math(EXPR final_length  "${name_length}-4") # remove .cpp of the name
  string(SUBSTRING ${_src_filename} 0 ${final_length} TEST_NAME)
  
  add_executable(${TEST_NAME}_test ${TEST_SOURCE} ${SOURCE_CPP_FILES})
  ament_target_dependencies(${TEST_NAME}_test  ${PROJECT_DEPENDENCIES})
  target_link_libraries(${TEST_NAME}_test benchmark::benchmark )


  endforeach()