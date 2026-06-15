cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME as2_behaviors_trajectory_generation)
project(${PROJECT_NAME} VERSION 1.0.0)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Set Release as default
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Set fPIC to ON by default
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Set commons dependencies
set(PROJECT_DEPENDENCIES
  ament_cmake
  pluginlib
  rclcpp
  rclcpp_action
  rclcpp_components
  std_msgs
  std_srvs
  as2_core
  as2_msgs
  as2_behavior
  as2_motion_reference_handlers
)

# Find dependencies
foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

# List of behaviors
set(BEHAVIORS_LIST
  generate_polynomial_trajectory_behavior
)

# Add subdirectories
foreach(BEHAVIOR ${BEHAVIORS_LIST})
  add_subdirectory(${BEHAVIOR})
endforeach()

# Pluginlib resource registry
pluginlib_export_plugin_description_file(${PROJECT_NAME}
  generate_polynomial_trajectory_behavior/plugins.xml)

ament_export_dependencies(${PROJECT_DEPENDENCIES})

# Build tests if testing is enabled
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # Keep ament linters out of generated / vendored trees.
  #
  # Two mechanisms are needed because they cover disjoint sets of linters:
  #   - AMENT_IGNORE markers: honored by cpplint, uncrustify, copyright,
  #     flake8, cppcheck, lint_cmake, xmllint, clang_format and by colcon
  #     for package discovery.
  #   - AMENT_LINT_AUTO_FILE_EXCLUDE: needed for ament_pep257, which does
  #     NOT honor AMENT_IGNORE but accepts directory paths via --exclude.
  #
  # We collect every directory to suppress in LINT_EXCLUDED_DIRS and feed
  # both mechanisms from that single list.
  set(LINT_EXCLUDED_DIRS)

  # 1. IDE artefacts left inside the source tree (e.g. VSCode CMake Tools
  #    with cmake.buildDirectory pointing at <pkg>/build). Plugins do not
  #    own these paths, so we drop the AMENT_IGNORE marker here.
  foreach(_DIR build install)
    set(_PATH ${CMAKE_CURRENT_SOURCE_DIR}/${_DIR})
    if(IS_DIRECTORY ${_PATH})
      file(WRITE ${_PATH}/AMENT_IGNORE "")
      list(APPEND LINT_EXCLUDED_DIRS ${_PATH})
    endif()
  endforeach()

  # 2. Vendored upstream sources under each plugin's thirdparties/. Each
  #    plugin's own CMakeLists already writes the AMENT_IGNORE marker
  #    inside its thirdparties/ at configure time, so we only need to
  #    extend the pep257 exclude list here.
  foreach(BEHAVIOR ${BEHAVIORS_LIST})
    file(GLOB _PLUGIN_THIRDPARTIES LIST_DIRECTORIES true
      ${CMAKE_CURRENT_SOURCE_DIR}/${BEHAVIOR}/plugins/*/thirdparties)
    list(APPEND LINT_EXCLUDED_DIRS ${_PLUGIN_THIRDPARTIES})
  endforeach()

  set(AMENT_LINT_AUTO_FILE_EXCLUDE ${LINT_EXCLUDED_DIRS})

  # ament_cmake_pep257 does not forward AMENT_LINT_AUTO_FILE_EXCLUDE to the
  # ament_pep257 binary, so we skip its auto-registration and re-register
  # the test manually with the explicit --exclude argv.
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_pep257)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_pep257 QUIET)
  if(ament_cmake_pep257_FOUND)
    if(AMENT_LINT_AUTO_FILE_EXCLUDE)
      ament_pep257(. --exclude ${AMENT_LINT_AUTO_FILE_EXCLUDE})
    else()
      ament_pep257()
    endif()
  endif()
endif()

ament_package()
