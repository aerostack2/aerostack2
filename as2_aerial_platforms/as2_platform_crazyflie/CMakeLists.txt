cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME as2_platform_crazyflie)
project(${PROJECT_NAME})

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
#########################


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(as2_core REQUIRED)
find_package(as2_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Eigen3 REQUIRED)


#download the crazyflie_cpp from github
set(USE_CRAZYFLIE_COMPLETE_LIB OFF)
find_package(crazyflie_cpp QUIET)
if (${crazyflie_cpp_FOUND})
  MESSAGE(STATUS "crazyflie_cpp found at ${crazyflie_cpp_DIR}")
  find_package(crazyflie_cpp REQUIRED)
  set(USE_CRAZYFLIE_COMPLETE_LIB ON)
else (${crazyflie_cpp_FOUND})
  MESSAGE(STATUS "crazyflie_cpp not found")
  include(FetchContent)
  FetchContent_Declare(
  crazyflie_cpp
  GIT_REPOSITORY https://github.com/miferco97/crazyflie_cpp.git
  # GIT_TAG 8b8391bc8e1899b4704723d22b5e4fa43ddef763
  GIT_TAG master
  )
  FetchContent_MakeAvailable(crazyflie_cpp)
endif(${crazyflie_cpp_FOUND})

set(HEADER_HPP_FILES
  include/crazyflie_platform.hpp
)

include_directories(
  include/${PROJECT_NAME}
  ${EIGEN3_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}_node src/crazyflie_node.cpp src/crazyflie_platform.cpp)
ament_target_dependencies(${PROJECT_NAME}_node 
  rclcpp 
  sensor_msgs 
  std_msgs
  std_srvs
  nav_msgs
  as2_core
  as2_msgs
  geometry_msgs
  Eigen3
)

if (${USE_CRAZYFLIE_COMPLETE_LIB})
  # message("Yes!! COMPLETE")
  target_link_libraries(${PROJECT_NAME}_node 
    crazyflie_cpp::crazyflie_cpp
  )
else (${USE_CRAZYFLIE_COMPLETE_LIB})
target_link_libraries(${PROJECT_NAME}_node 
  crazyflie_cpp
)
endif()


add_executable(${PROJECT_NAME}_swarm_node src/crazyflie_swarm_nodes.cpp src/crazyflie_platform.cpp)
ament_target_dependencies(${PROJECT_NAME}_swarm_node 
  rclcpp 
  sensor_msgs 
  std_msgs
  std_srvs
  nav_msgs
  as2_core
  as2_msgs
  geometry_msgs
  Eigen3
)

if (${USE_CRAZYFLIE_COMPLETE_LIB})
  # message("Yes!! COMPLETE")
  target_link_libraries(${PROJECT_NAME}_swarm_node 
    crazyflie_cpp::crazyflie_cpp
  )
else (${USE_CRAZYFLIE_COMPLETE_LIB})
target_link_libraries(${PROJECT_NAME}_swarm_node 
  crazyflie_cpp
)
endif()



if(BUILD_TESTING)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)
  
  ament_cppcheck(src/ include/ tests/)
  ament_clang_format(src/ include/ tests/ --config ${CMAKE_CURRENT_SOURCE_DIR}/.clang-format)
endif()


install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
${PROJECT_NAME}_node
${PROJECT_NAME}_swarm_node
  DESTINATION lib/${PROJECT_NAME})


# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/aideck_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
