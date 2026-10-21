find_package(ament_cmake_gtest REQUIRED)

file(GLOB GTEST_SOURCES *_gtest.cpp)

foreach(GTEST_SOURCE ${GTEST_SOURCES})
  get_filename_component(TEST_NAME ${GTEST_SOURCE} NAME_WE)

  ament_add_gtest(${TEST_NAME} ${GTEST_SOURCE})
  ament_target_dependencies(${TEST_NAME} ${PROJECT_DEPENDENCIES})
  target_link_libraries(${TEST_NAME} ${PROJECT_NAME})
endforeach()
