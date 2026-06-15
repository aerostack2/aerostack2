# Behavior-level tests: bring up the wrapper node with the default plugin
# and verify it constructs without throwing.
file(GLOB GTEST_SOURCES "*_gtest.cpp")

if(GTEST_SOURCES)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_index_cpp REQUIRED)

  foreach(TEST_SOURCE ${GTEST_SOURCES})
    get_filename_component(TEST_NAME ${TEST_SOURCE} NAME_WE)

    ament_add_gtest(${PROJECT_NAME}_${TEST_NAME} ${TEST_SOURCE})
    ament_target_dependencies(${PROJECT_NAME}_${TEST_NAME}
      ${PROJECT_DEPENDENCIES}
      ${BEHAVIOR_DEPENDENCIES})
    target_link_libraries(${PROJECT_NAME}_${TEST_NAME}
      ament_index_cpp::ament_index_cpp
      gtest_main
      ${BEHAVIOR_NAME}
      ${BEHAVIOR_NAME}_plugin_base)
  endforeach()
endif()

# Integration tests: single-process executables that boot the behavior + a
# mock support node + an action client and exercise pause/resume/modify
# scenarios end-to-end. Parametrized internally over every pluginlib-declared
# trajectory generator plugin.
file(GLOB INTEGRATION_TEST_SOURCES "*_test.cpp")

if(INTEGRATION_TEST_SOURCES)
  find_package(GTest REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  find_package(tf2_ros REQUIRED)

  foreach(TEST_SOURCE ${INTEGRATION_TEST_SOURCES})
    get_filename_component(TEST_NAME ${TEST_SOURCE} NAME_WE)

    add_executable(${PROJECT_NAME}_${TEST_NAME} ${TEST_SOURCE})
    ament_target_dependencies(${PROJECT_NAME}_${TEST_NAME}
      ${PROJECT_DEPENDENCIES}
      ${BEHAVIOR_DEPENDENCIES}
      tf2_ros)
    target_link_libraries(${PROJECT_NAME}_${TEST_NAME}
      ament_index_cpp::ament_index_cpp
      GTest::gtest
      ${BEHAVIOR_NAME}
      ${BEHAVIOR_NAME}_plugin_base)

    add_test(
      NAME ${PROJECT_NAME}_${TEST_NAME}
      COMMAND ${PROJECT_NAME}_${TEST_NAME})
    set_tests_properties(${PROJECT_NAME}_${TEST_NAME}
      PROPERTIES TIMEOUT 120 LABELS "integration")
  endforeach()
endif()

# Google-Benchmark comparative suite: one executable that loads every
# plugin via pluginlib and times generateTrajectory / updateWaypoints /
# evaluate as a function of the mission waypoint count. Not registered as
# a CTest target; the user runs it manually from build/ to choose a
# backend. The dependency is intentionally optional — if Google Benchmark
# is not installed, the rest of the package builds unchanged.
find_package(benchmark QUIET)
if(benchmark_FOUND)
  set(BM_NAME ${PROJECT_NAME}_${BEHAVIOR_NAME}_benchmark)
  add_executable(${BM_NAME}
    generate_polynomial_trajectory_behavior_benchmark.cpp)
  target_include_directories(${BM_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/common)
  ament_target_dependencies(${BM_NAME}
    ${PROJECT_DEPENDENCIES}
    ${BEHAVIOR_DEPENDENCIES})
  target_link_libraries(${BM_NAME}
    benchmark::benchmark
    ${BEHAVIOR_NAME}_plugin_base)
else()
  message(STATUS
    "Google Benchmark not found; skipping comparative benchmark target.")
endif()
