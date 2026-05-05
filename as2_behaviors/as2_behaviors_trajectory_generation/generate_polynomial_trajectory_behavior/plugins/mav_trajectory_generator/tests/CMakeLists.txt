# GTest of the mav_trajectory_generator plugin.
# Loads the plugin via pluginlib and exercises the public base API.
file(GLOB GTEST_SOURCES "*_gtest.cpp")
file(GLOB BENCHMARK_SOURCES "*_benchmark.cpp")

if(GTEST_SOURCES)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_index_cpp REQUIRED)

  foreach(TEST_SOURCE ${GTEST_SOURCES})
    get_filename_component(TEST_NAME ${TEST_SOURCE} NAME_WE)

    ament_add_gtest(${TEST_NAME} ${TEST_SOURCE})
    ament_target_dependencies(${TEST_NAME} ${PLUGIN_DEPENDENCIES})
    target_link_libraries(${TEST_NAME}
      ament_index_cpp::ament_index_cpp
      gtest_main
      ${PLUGIN_NAME}
      ${BEHAVIOR_NAME}_plugin_base
    )
  endforeach()
endif()

# Per-plugin Google-Benchmark binaries. Each source owns its main() and
# registers whichever plugin it wants to exercise.
find_package(benchmark QUIET)
if(benchmark_FOUND AND BENCHMARK_SOURCES)
  set(BENCHMARK_COMMON_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../tests/common)

  foreach(BENCHMARK_SOURCE ${BENCHMARK_SOURCES})
    get_filename_component(BENCHMARK_NAME ${BENCHMARK_SOURCE} NAME_WE)

    add_executable(${PROJECT_NAME}_${BENCHMARK_NAME} ${BENCHMARK_SOURCE})
    target_include_directories(${PROJECT_NAME}_${BENCHMARK_NAME} PRIVATE
      ${BENCHMARK_COMMON_DIR})
    ament_target_dependencies(${PROJECT_NAME}_${BENCHMARK_NAME}
      ${PLUGIN_DEPENDENCIES})
    target_link_libraries(${PROJECT_NAME}_${BENCHMARK_NAME}
      benchmark::benchmark
      ${PLUGIN_NAME}
      ${BEHAVIOR_NAME}_plugin_base
    )
  endforeach()
endif()
