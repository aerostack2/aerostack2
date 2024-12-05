cmake_minimum_required(VERSION 3.8)
set (PROJECT_NAME as2_behavior_swarm_msgs)
project(${PROJECT_NAME}  VERSION 1.0.0)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(PROJECT_DEPENDENCIES
  as2_msgs
  geometry_msgs
  )

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

set(SRVS_FILES
  "srv/StartSwarm.srv"
)
set(MSG_FILES

)
set(ACTION_FILES
  "action/Swarm.action"
)


rosidl_generate_interfaces(${PROJECT_NAME}
  # ${MSGS_FILES}
  ${SRVS_FILES}
  ${ACTION_FILES}
  DEPENDENCIES ${PROJECT_DEPENDENCIES}
)
if(BUILD_TESTING)
  find_package(ament_cmake_lint_cmake REQUIRED)
  ament_lint_cmake()
  find_package(ament_cmake_xmllint REQUIRED)
  ament_xmllint()
endif()

ament_package()
