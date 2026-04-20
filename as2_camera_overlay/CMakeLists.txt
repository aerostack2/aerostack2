cmake_minimum_required(VERSION 3.10)
project(as2_camera_overlay VERSION 0.1.0)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_compile_options(-Wall -Wextra)
if(CMAKE_BUILD_TYPE STREQUAL "Release" OR CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
  add_compile_options(-O3)
endif()

# AUTOMOC is required for HeadlessDisplayContext and HeadlessFrameManager (Q_OBJECT).
set(CMAKE_AUTOMOC ON)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_components
  pluginlib
  sensor_msgs
  geometry_msgs
  visualization_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  image_transport
  cv_bridge
  rviz_rendering
  rviz_common
  rviz_default_plugins
  rviz_ogre_vendor
  ament_index_cpp
)

find_package(ament_cmake REQUIRED)
foreach(pkg ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${pkg} REQUIRED)
endforeach()

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Core Widgets Gui)

set(OVERLAY_CORE_SRC
  src/camera_projection.cpp
  src/frame_helpers.cpp
  src/overlay_renderer.cpp
  src/tf2_buffer_frame_transformer.cpp
  src/headless_ros_node.cpp
  src/headless_frame_manager.cpp
  src/headless_display_context.cpp
  src/display_loader.cpp
  # Headers with Q_OBJECT must be listed for AUTOMOC to process them
  include/headless_frame_manager.hpp
  include/headless_display_context.hpp
)

add_library(${PROJECT_NAME}_core SHARED ${OVERLAY_CORE_SRC})
target_include_directories(${PROJECT_NAME}_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_core
  PUBLIC
    Qt5::Core
    Qt5::Widgets
    Qt5::Gui
    ${OpenCV_LIBS}
    Eigen3::Eigen
    yaml-cpp
)
ament_target_dependencies(${PROJECT_NAME}_core
  PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

add_executable(${PROJECT_NAME}_node
  src/main.cpp
  src/overlay_node.cpp
)
target_include_directories(${PROJECT_NAME}_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_node
  PUBLIC
    ${PROJECT_NAME}_core
    Qt5::Core
    Qt5::Widgets
    Qt5::Gui
    ${OpenCV_LIBS}
    Eigen3::Eigen
    yaml-cpp
)
ament_target_dependencies(${PROJECT_NAME}_node
  PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(DIRECTORY include/ DESTINATION include)

install(
  TARGETS ${PROJECT_NAME}_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS} OpenCV Eigen3)

ament_package()
