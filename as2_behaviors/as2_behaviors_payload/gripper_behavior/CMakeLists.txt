set(SUBPROJECT_NAME  gripper_behavior)

# Set commons dependencies (inherited from parent + specific ones)
set(SUBPROJECT_DEPENDENCIES
  ${PROJECT_DEPENDENCIES}
)

# Find dependencies
foreach(DEPENDENCY ${SUBPROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

# Include directories
include_directories(
  include
  include/${SUBPROJECT_NAME}
)

# Create plugin base library
set(PLUGIN_BASE_FILES
  include/${SUBPROJECT_NAME}/${SUBPROJECT_NAME}_plugin_base.hpp
)

add_library(${SUBPROJECT_NAME}_plugin_base SHARED ${PLUGIN_BASE_FILES})
set_target_properties(${SUBPROJECT_NAME}_plugin_base PROPERTIES LINKER_LANGUAGE CXX)
ament_target_dependencies(${SUBPROJECT_NAME}_plugin_base ${SUBPROJECT_DEPENDENCIES})

# Create main behavior library
set(BEHAVIOR_SOURCE_FILES
  src/${SUBPROJECT_NAME}.cpp
)

add_library(${SUBPROJECT_NAME}_lib SHARED ${BEHAVIOR_SOURCE_FILES})
ament_target_dependencies(${SUBPROJECT_NAME}_lib ${SUBPROJECT_DEPENDENCIES})
target_link_libraries(${SUBPROJECT_NAME}_lib
  ${SUBPROJECT_NAME}_plugin_base
)

# Create executable
set(NODE_SOURCE_FILES
  src/${SUBPROJECT_NAME}_node.cpp
)

add_executable(${SUBPROJECT_NAME}_node ${NODE_SOURCE_FILES})
target_link_libraries(${SUBPROJECT_NAME}_node
  ${SUBPROJECT_NAME}_lib
)
ament_target_dependencies(${SUBPROJECT_NAME}_node ${SUBPROJECT_DEPENDENCIES})

target_include_directories(${SUBPROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Export libraries and targets
install(
  TARGETS
    ${SUBPROJECT_NAME}_lib
    ${SUBPROJECT_NAME}_plugin_base
  EXPORT export_${SUBPROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  TARGETS ${SUBPROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# Export headers
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}/${SUBPROJECT_NAME}
)
# Add Plugins
set(PLUGIN_LIST
  two_fingers
  dc_servo
)

foreach(PLUGIN ${PLUGIN_LIST})
  add_subdirectory(plugins/${PLUGIN})
endforeach()

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/${SUBPROJECT_NAME}/launch
)

# Install plugin configs
foreach(PLUGIN ${PLUGIN_LIST})
  install(
    DIRECTORY plugins/${PLUGIN}/config/
    DESTINATION share/${PROJECT_NAME}/${SUBPROJECT_NAME}/plugins/${PLUGIN}/config
  )
endforeach()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests)
    add_subdirectory(tests)
  endif()
endif()

ament_export_dependencies(${SUBPROJECT_DEPENDENCIES})
ament_export_targets(export_${SUBPROJECT_NAME})

# Note: ament_package() is only called in the main CMakeLists.txt, not here