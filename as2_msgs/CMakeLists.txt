cmake_minimum_required(VERSION 3.5)

set(PROJECT_NAME as2_msgs)
project(${PROJECT_NAME} VERSION 1.0.0)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(PROJECT_DEPENDENCIES
  std_msgs
  geometry_msgs
  geographic_msgs
  builtin_interfaces
  nav_msgs
  action_msgs
  sensor_msgs
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()


file(GLOB_RECURSE MSGS_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "msg/*.msg")

file(GLOB_RECURSE SRVS_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "srv/*.srv")

file(GLOB_RECURSE ACTION_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "action/*.action")

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSGS_FILES}
  ${SRVS_FILES}
  ${ACTION_FILES}
  DEPENDENCIES ${PROJECT_DEPENDENCIES}
)

# macOS + conda/pixi workaround for rosidl-generated shared libraries.
#
# On Linux, LD_LIBRARY_PATH is set by the workspace setup script so that
# dlopen() can find message libraries installed under install/<pkg>/lib/.
# On macOS, the equivalent DYLD_LIBRARY_PATH is silently stripped by System
# Integrity Protection (SIP), which means the ROS 2 middleware (e.g.
# rmw_cyclonedds) cannot dlopen() locally-built message type support libraries
# by filename alone — they are simply not on any search path that dyld knows
# about.
#
# To work around this, we create symlinks from the conda/pixi environment lib
# directory (which dyld does search) pointing to the actual libraries in the
# colcon install directory. This runs at install time so the symlinks are
# always up-to-date after a rebuild.
if(APPLE AND DEFINED ENV{CONDA_PREFIX})
  set(_conda_lib "$ENV{CONDA_PREFIX}/lib")
  install(CODE "
    file(GLOB _libs \"\${CMAKE_INSTALL_PREFIX}/lib/lib${PROJECT_NAME}*.dylib\")
    foreach(_lib \${_libs})
      get_filename_component(_name \${_lib} NAME)
      set(_link \"${_conda_lib}/\${_name}\")
      if(NOT EXISTS \${_link})
        execute_process(COMMAND \${CMAKE_COMMAND} -E create_symlink \${_lib} \${_link})
        message(STATUS \"Symlinked \${_name} -> ${_conda_lib}\")
      endif()
    endforeach()
  ")
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_lint_cmake REQUIRED)
  ament_lint_cmake()
  find_package(ament_cmake_xmllint REQUIRED)
  ament_xmllint()
endif()

ament_package()
