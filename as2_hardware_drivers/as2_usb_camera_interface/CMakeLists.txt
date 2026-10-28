cmake_minimum_required(VERSION 3.5)

set(PROJECT_NAME as2_usb_camera_interface)
project(${PROJECT_NAME} VERSION 0.1.0)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# set Release as default
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# find dependencies
set(PROJECT_DEPENDENCIES
  ament_cmake
  rclcpp
  as2_core
  as2_msgs
  geometry_msgs
  std_msgs
  std_srvs
  sensor_msgs
  OpenCV
  cv_bridge
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

# remap_with_cuda remaps on the GPU with NPP from the CUDA toolkit. Without it, it falls back to the
# CPU.
find_package(CUDAToolkit QUIET)
if(TARGET CUDA::nppig)
  message(STATUS "remap_with_cuda: GPU remap with NPP (CUDA ${CUDAToolkit_VERSION})")
  set(NOMINAL_REMAP_GPU AS2_USB_CAMERA_INTERFACE_NPP)
  set(NOMINAL_REMAP_GPU_LIBRARIES CUDA::nppig CUDA::nppisu CUDA::nppc CUDA::cudart)
else()
  message(STATUS "remap_with_cuda: no NPP (CUDA toolkit), falls back to the CPU")
  set(NOMINAL_REMAP_GPU "")
  set(NOMINAL_REMAP_GPU_LIBRARIES "")
endif()

# include directories
include_directories(
  include
  include/${PROJECT_NAME}
)

# add header files
set(HEADER_HPP_FILES
  include/${PROJECT_NAME}/as2_usb_camera_interface.hpp
  include/${PROJECT_NAME}/common.hpp
  include/${PROJECT_NAME}/nominal_remap.hpp
)

# add source files
set(SOURCE_CPP_FILES
  src/as2_usb_camera_interface.cpp
  src/nominal_remap.cpp
)

# Create the node executable
add_executable(${PROJECT_NAME}_node src/as2_usb_camera_interface_node.cpp ${SOURCE_CPP_FILES})
ament_target_dependencies(${PROJECT_NAME}_node ${PROJECT_DEPENDENCIES})
target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_node yaml-cpp::yaml-cpp ${NOMINAL_REMAP_GPU_LIBRARIES})
target_compile_definitions(${PROJECT_NAME}_node PRIVATE ${NOMINAL_REMAP_GPU})

# Create the library
add_library(${PROJECT_NAME} STATIC ${SOURCE_CPP_FILES})
ament_target_dependencies(${PROJECT_NAME} ${PROJECT_DEPENDENCIES})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} yaml-cpp::yaml-cpp ${NOMINAL_REMAP_GPU_LIBRARIES})
target_compile_definitions(${PROJECT_NAME} PRIVATE ${NOMINAL_REMAP_GPU})

# Install the headers
install(
  DIRECTORY include/
  DESTINATION include
)

# install the executable
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install the node executable
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

# Install the library
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${PROJECT_DEPENDENCIES})
if(NOMINAL_REMAP_GPU_LIBRARIES)
  # The static library's users link NPP too
  ament_export_dependencies(CUDAToolkit)
endif()

# Build tests if testing is enabled
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  file(GLOB_RECURSE EXCLUDE_FILES
    build/*
    install/*
  )
  set(AMENT_LINT_AUTO_FILE_EXCLUDE ${EXCLUDE_FILES})
  ament_lint_auto_find_test_dependencies()

  add_subdirectory(tests)
endif()

# Create ament package
ament_package()
