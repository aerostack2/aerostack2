find_package(ament_cmake_gtest REQUIRED)

# Path Optimizer test
ament_add_gtest(test_path_optimizer tests/test_path_optimizer.cpp)
ament_target_dependencies(test_path_optimizer ${PROJECT_DEPENDENCIES})
target_link_libraries(test_path_optimizer gtest_main ${PROJECT_NAME})

# Explorer test
ament_add_gtest(test_explorer tests/test_explorer.cpp)
ament_target_dependencies(test_explorer ${PROJECT_DEPENDENCIES})
target_link_libraries(test_explorer gtest_main ${PROJECT_NAME})