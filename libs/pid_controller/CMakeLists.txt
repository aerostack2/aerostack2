cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME pid_controller)
project(${PROJECT_NAME})

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# set Release as default
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

#set fPIC to ON by default
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# find dependencies
set(PROJECT_DEPENDENCIES
  ament_cmake
  Eigen3
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

include_directories(
  include
  include/${PROJECT_NAME}

  # uncomment if using Eigen3
  ${EIGEN3_INCLUDE_DIRS}
)

set(SOURCE_CPP_FILES
  src/PID.cpp
  src/PID_3D.cpp
)

# create library and export it
add_library(${PROJECT_NAME} ${SOURCE_CPP_FILES})

# message("Building ${PROJECT_NAME}")
target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME} ${PROJECT_DEPENDENCIES})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${PROJECT_DEPENDENCIES})

# ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

# target_include_directories(${PROJECT_NAME} PUBLIC include include/${PROJECT_NAME} ${EIGEN3_INCLUDE_DIRS} ${${PROJECT_DEPENDENCIES}})

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin

  INCLUDES DESTINATION include
)

# check if tests are enabled
if(BUILD_TESTING)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)

  ament_cppcheck(src/ include/ tests/)
  ament_clang_format(src/ include/ tests/ --config ${CMAKE_CURRENT_SOURCE_DIR}/.clang-format)

  include(tests/profiling_cmake.txt)
endif()

ament_package()