cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME ekf)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
project(${PROJECT_NAME})

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Set Release as default
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Find dependencies
set(PROJECT_DEPENDENCIES
  ament_cmake
  eigen3_cmake_module
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
  message(STATUS "Found ${DEPENDENCY}: ${${DEPENDENCY}_FOUND}")
endforeach()

find_package(Eigen3 REQUIRED NO_MODULE)

message(STATUS "Using Eigen3: ${EIGEN3_INCLUDE_DIRS}")

set(SOURCE_CPP_FILES
  src/ekf_c_code.cpp
  src/ekf_datatype.cpp
  src/ekf_wrapper.cpp
)

# include_directories(
#   include
#   include/${PROJECT_NAME}
#   ${EIGEN3_INCLUDE_DIRS}
# )

# Create the library
add_library(${PROJECT_NAME} SHARED ${SOURCE_CPP_FILES})

# Export library headers
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<TARGET_PROPERTY:Eigen3::Eigen,INTERFACE_INCLUDE_DIRECTORIES>
)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

# Export libraries and targets
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# install(TARGETS
#   ${PROJECT_NAME}_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# Export headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Add executables for each example
if(BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()

# ament_target_dependencies(${PROJECT_NAME} ${PROJECT_DEPENDENCIES})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(Eigen3)

ament_package()
