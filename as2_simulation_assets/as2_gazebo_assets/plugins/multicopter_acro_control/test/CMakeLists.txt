#Gtest

#find dependencies

set(PROJECT_DEPENDENCIES
    gz-sim8
    gz-common5
    gz-math7
    gz-transport13
    gz-plugin2
    gz-rendering8
    gz-msgs10
)

set(TEST_DEPENDENCIES
    ament_cmake_gtest
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
    find_package(${DEPENDENCY} REQUIRED)
endforeach()

find_package(Eigen3 3.3 REQUIRED)

foreach(DEPENDENCY ${TEST_DEPENDENCIES})
    find_package(${DEPENDENCY} REQUIRED)
endforeach()

# Google Test tests
file(GLOB_RECURSE GTEST_SOURCE "*_gtest.cpp")

if(GTEST_SOURCE)
  find_package(GTest REQUIRED)

  foreach(TEST_FILE ${GTEST_SOURCE})
    # get_filename_component(TEST_NAME ${TEST_FILE} NAME_WE)

    add_executable(${PROJECT_NAME}_${TEST_NAME} ${TEST_FILE})
    target_link_libraries(${PROJECT_NAME}_${TEST_NAME} GTest::gtest MulticopterACROControl ${PROJECT_DEPENDENCIES})

    add_test(
      NAME ${PROJECT_NAME}_${TEST_NAME}
      COMMAND ${PROJECT_NAME}_${TEST_NAME}
    )

    install(
    TARGETS ${PROJECT_NAME}_${TEST_NAME}
    DESTINATION lib)
  endforeach()
endif()