set(BEHAVIOR_NAME generate_polynomial_trajectory_behavior)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
link_libraries(Eigen3::Eigen)
add_compile_options("$<$<COMPILE_LANGUAGE:CXX>:-include$<SEMICOLON>cassert>" "$<$<COMPILE_LANGUAGE:C>:-include$<SEMICOLON>assert.h>")

# Behavior-level dependencies (in addition to the package-level ones).
set(BEHAVIOR_DEPENDENCIES
  trajectory_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  Eigen3
  rclcpp_components
)
foreach(DEPENDENCY ${BEHAVIOR_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

include_directories(
  include
  include/${BEHAVIOR_NAME}
  ${EIGEN3_INCLUDE_DIRS}
)

# --- Plugin base library ---------------------------------------------------
# Holds the abstract interface and the shared implementation (debug
# publishers, yaw computation, vehicle state, evaluation orchestration).
add_library(${BEHAVIOR_NAME}_plugin_base SHARED
  src/generate_polynomial_trajectory_base.cpp)
target_include_directories(${BEHAVIOR_NAME}_plugin_base PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${BEHAVIOR_NAME}_plugin_base
  ${PROJECT_DEPENDENCIES}
  ${BEHAVIOR_DEPENDENCIES})

# --- Wrapper server (BehaviorServer + composable component) ---------------
add_library(${BEHAVIOR_NAME} SHARED src/${BEHAVIOR_NAME}.cpp)
target_link_libraries(${BEHAVIOR_NAME} ${BEHAVIOR_NAME}_plugin_base)
ament_target_dependencies(${BEHAVIOR_NAME}
  ${PROJECT_DEPENDENCIES}
  ${BEHAVIOR_DEPENDENCIES})
rclcpp_components_register_nodes(${BEHAVIOR_NAME} "GeneratePolynomialTrajectoryBehavior")

# --- Standalone executable -------------------------------------------------
add_executable(${BEHAVIOR_NAME}_node src/${BEHAVIOR_NAME}_node.cpp)
target_link_libraries(${BEHAVIOR_NAME}_node ${BEHAVIOR_NAME})
ament_target_dependencies(${BEHAVIOR_NAME}_node
  ${PROJECT_DEPENDENCIES}
  ${BEHAVIOR_DEPENDENCIES})

# --- Install ---------------------------------------------------------------
install(TARGETS
    ${BEHAVIOR_NAME}
    ${BEHAVIOR_NAME}_plugin_base
  EXPORT export_${BEHAVIOR_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(TARGETS
  ${BEHAVIOR_NAME}_node
  DESTINATION
  lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION
  include
)

ament_export_include_directories(include)
ament_export_libraries(${BEHAVIOR_NAME} ${BEHAVIOR_NAME}_plugin_base)
ament_export_targets(export_${BEHAVIOR_NAME})

# --- Plugins ---------------------------------------------------------------
set(PLUGIN_LIST
  dynamic_mav_trajectory_generator
  mav_trajectory_generator
  gcopter_trajectory_generator
  jerk_limited_trajectory_generator
)

foreach(PLUGIN ${PLUGIN_LIST})
  add_subdirectory(plugins/${PLUGIN})
endforeach()

# Pluginlib registry — must be exported once per package, pointing to the
# plugins.xml that lists every plugin of this behavior. The actual call lives
# in the root CMakeLists.txt because pluginlib_export_plugin_description_file
# accumulates state in a variable that does not propagate from a subdirectory
# to the project's ament_package() call.

foreach(PLUGIN ${PLUGIN_LIST})
  install(
    DIRECTORY plugins/${PLUGIN}/config
    DESTINATION share/${PROJECT_NAME}/${BEHAVIOR_NAME}/plugins/${PLUGIN}
  )
endforeach()

# --- Launch + config -------------------------------------------------------
install(DIRECTORY
  launch
  DESTINATION
  share/${PROJECT_NAME}/${BEHAVIOR_NAME}
)

install(DIRECTORY
  config
  DESTINATION
  share/${PROJECT_NAME}/${BEHAVIOR_NAME}
)


# --- Tests -----------------------------------------------------------------
if(BUILD_TESTING)
  add_subdirectory(tests)
endif()
