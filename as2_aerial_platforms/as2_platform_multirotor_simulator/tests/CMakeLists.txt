# Tests
file(GLOB TEST_SOURCE "*_test.cpp")

if(TEST_SOURCE)
  foreach(TEST_FILE ${TEST_SOURCE})
    get_filename_component(TEST_NAME ${TEST_FILE} NAME_WE)

    add_executable(${PROJECT_NAME}_${TEST_NAME} ${TEST_FILE})
    ament_target_dependencies(${PROJECT_NAME}_${TEST_NAME} ${PROJECT_DEPENDENCIES})
    target_link_libraries(${PROJECT_NAME}_${TEST_NAME} ${PROJECT_NAME})
  endforeach()
endif()

# GTest
file(GLOB GTEST_SOURCE "*_gtest.cpp")

if(GTEST_SOURCE)
  find_package(ament_cmake_gtest REQUIRED)

  foreach(TEST_SOURCE ${GTEST_SOURCE})
    get_filename_component(TEST_NAME ${TEST_SOURCE} NAME_WE)

    ament_add_gtest(${PROJECT_NAME}_${TEST_NAME} ${TEST_SOURCE})
    ament_target_dependencies(${PROJECT_NAME}_${TEST_NAME} ${PROJECT_DEPENDENCIES})
    target_link_libraries(${PROJECT_NAME}_${TEST_NAME} gtest_main ${PROJECT_NAME})
  endforeach()
endif()

# Benchmark
file(GLOB BENCHMARK_SOURCE "*_benchmark.cpp")

if(BENCHMARK_SOURCE)
  find_package(benchmark REQUIRED)

  foreach(BENCHMARK_FILE ${BENCHMARK_SOURCE})
    get_filename_component(BENCHMARK_NAME ${BENCHMARK_FILE} NAME_WE)

    add_executable(${PROJECT_NAME}_${BENCHMARK_NAME} ${BENCHMARK_FILE})
    target_link_libraries(${PROJECT_NAME}_${BENCHMARK_NAME} ${PROJECT_NAME} benchmark::benchmark)
  endforeach()
endif()