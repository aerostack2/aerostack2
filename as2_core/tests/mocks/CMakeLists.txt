# File to include, use parent CMakelists path

# Add mocks library
set(MOCK_SOURCES
  tests/mocks/aerial_platform/mock_aerial_platform.cpp
  tests/mocks/executor_thread_util/executor_thread_util.cpp
)

add_library(${MOCKS_LIBRARY_NAME} SHARED ${MOCK_SOURCES})
ament_target_dependencies(${MOCKS_LIBRARY_NAME} ${PROJECT_DEPENDENCIES})

install(
  DIRECTORY tests/mocks/
  DESTINATION include/${PROJECT_NAME}/${MOCKS_LIBRARY_NAME}/
  FILES_MATCHING PATTERN "*.hpp"
)