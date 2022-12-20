# find all *.cpp files in the tests directory
file(GLOB TEST_SOURCES tests/px4_odom.cpp)

set(TEST_NAME odom)

# create a test executable for each test file
foreach(TEST_SOURCE ${TEST_SOURCES})
    add_executable(${TEST_NAME}_test ${TEST_SOURCE} src/pixhawk_platform.cpp)
    ament_target_dependencies(${TEST_NAME}_test ${PROJECT_DEPENDENCIES})
    target_link_libraries(${TEST_NAME}_test)
endforeach()
