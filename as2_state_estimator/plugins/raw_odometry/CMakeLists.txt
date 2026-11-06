cmake_minimum_required(VERSION 3.5)
# Deprecated alias of external_localization, kept so existing raw_odometry configs keep working
set(PLUGIN_NAME raw_odometry)

add_library(${PLUGIN_NAME} SHARED src/${PLUGIN_NAME}.cpp)
target_link_libraries(${PLUGIN_NAME} external_localization ${PROJECT_NAME} ${PROJECT_NAME}_plugin_base)
target_include_directories(${PLUGIN_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../external_localization/include)
ament_target_dependencies(${PLUGIN_NAME} pluginlib)

install(
  TARGETS ${PLUGIN_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()
