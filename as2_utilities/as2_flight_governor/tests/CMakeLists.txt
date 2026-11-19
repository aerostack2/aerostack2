find_package(ament_cmake_gtest REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Core: no ROS needed
ament_add_gtest(${PROJECT_NAME}_core_gtest governor_core_gtest.cpp)
target_link_libraries(${PROJECT_NAME}_core_gtest ${PROJECT_NAME}_core)

ament_add_gtest(${PROJECT_NAME}_loader_gtest policy_loader_gtest.cpp)
target_link_libraries(${PROJECT_NAME}_loader_gtest ${PROJECT_NAME}_core)
target_compile_definitions(${PROJECT_NAME}_loader_gtest PRIVATE
  CONFIG_DIR="${PROJECT_SOURCE_DIR}/config")

# Node: governs references end to end over ROS topics
ament_add_gtest(${PROJECT_NAME}_node_gtest flight_governor_gtest.cpp
  TIMEOUT 120 SKIP_LINKING_MAIN_LIBRARIES)
target_link_libraries(${PROJECT_NAME}_node_gtest ${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME}_node_gtest ${DEPENDENCIES})
target_compile_definitions(${PROJECT_NAME}_node_gtest PRIVATE
  CONFIG_DIR="${PROJECT_SOURCE_DIR}/config")
