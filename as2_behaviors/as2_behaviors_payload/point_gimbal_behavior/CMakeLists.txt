set(SUBPROJECT_NAME point_gimbal_behavior)

# Find dependencies
set(EXECUTABLE_DEPENDENCIES
  sensor_msgs
)

foreach(DEPENDENCY ${EXECUTABLE_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

# Include directories
include_directories(
  include
  include/${SUBPROJECT_NAME}
)

set(SOURCE_CPP_FILES
  src/${SUBPROJECT_NAME}.cpp
  src/${SUBPROJECT_NAME}_node.cpp
)

add_executable(${SUBPROJECT_NAME}_node ${SOURCE_CPP_FILES})
ament_target_dependencies(${SUBPROJECT_NAME}_node ${PROJECT_DEPENDENCIES} ${EXECUTABLE_DEPENDENCIES})

# Library
add_library(${SUBPROJECT_NAME} SHARED ${SOURCE_CPP_FILES})
ament_target_dependencies(${SUBPROJECT_NAME} ${PROJECT_DEPENDENCIES})

# Include headers in the library
target_include_directories(${SUBPROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install
install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/${SUBPROJECT_NAME})

install(TARGETS
  ${SUBPROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${SUBPROJECT_NAME}
)

ament_export_targets(
  export_${SUBPROJECT_NAME}
)

install(
  TARGETS ${SUBPROJECT_NAME}
  EXPORT export_${SUBPROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()