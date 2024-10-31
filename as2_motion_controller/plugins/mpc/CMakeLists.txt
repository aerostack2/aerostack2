cmake_minimum_required(VERSION 3.5)
set(PLUGIN_NAME mpc)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# set Release as default
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# set fPIC to ON by default
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# find dependencies
set(PLUGIN_DEPENDENCIES
  ament_cmake
  rclcpp
  pluginlib
  as2_core
  as2_msgs
  as2_motion_reference_handlers
  geometry_msgs
  Eigen3
)

foreach(DEPENDENCY ${PLUGIN_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

if(${acados_mpc_FOUND})
  message(STATUS "acados_mpc found")
else()
  message(STATUS "acados_mpc not found")
  include(FetchContent)
  fetchcontent_declare(
    acados_mpc
    GIT_REPOSITORY https://github.com/RPS98/mpc.git
    GIT_TAG main
  )
  fetchcontent_makeavailable(acados_mpc)
endif()

include_directories(
  include
  include/${PLUGIN_NAME}
  ${EIGEN3_INCLUDE_DIRS}

  # Include mpc
  ${CMAKE_BINARY_DIR}/_deps/mpc-src/include
)

set(SOURCE_CPP_FILES
  src/${PLUGIN_NAME}.cpp
)

# Library
add_library(${PLUGIN_NAME} SHARED ${SOURCE_CPP_FILES})
target_link_libraries(${PLUGIN_NAME} ${PROJECT_NAME} ${PROJECT_NAME}_plugin_base acados_mpc)
ament_target_dependencies(${PLUGIN_NAME} ${PLUGIN_DEPENDENCIES})

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PLUGIN_NAME}
)
ament_export_targets(
  export_${PLUGIN_NAME}
)

install(
  TARGETS ${PLUGIN_NAME}
  EXPORT export_${PLUGIN_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

message(STATUS "Acados Libs: ${ACADOS_LIB}")

# Install acados shared library
install(
  FILES ${CMAKE_BINARY_DIR}/_deps/acados_mpc-build/libacados_mpc.so
  DESTINATION lib
)

install(
  DIRECTORY $ENV{ACADOS_SOURCE_DIR}/lib/
  DESTINATION lib
)

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()