cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME as2_platform_dji_osdk)
project(${PROJECT_NAME})

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

set(PROJECT_DEPENDENCIES
rclcpp
std_msgs
sensor_msgs
geometry_msgs
nav_msgs
as2_core
as2_msgs
std_srvs
)

foreach(project ${PROJECT_DEPENDENCIES})
  find_package(${project} REQUIRED)
  message(STATUS "Found ${project}")
endforeach()

if(CMAKE_SYSTEM_PROCESSOR MATCHES "i386|i686|x86|AMD64|x86_64")
   set(ARCH x86)
   add_definitions(-DDJIOSDK_HARDWARE_TYPE=3)
elseif((CMAKE_SYSTEM_PROCESSOR MATCHES "^arm*") OR (CMAKE_SYSTEM_PROCESSOR MATCHES "^aarch"))
  if(CMAKE_SIZEOF_VOID_P EQUAL 4)
     set(ARCH armv7)
     add_definitions(-DDJIOSDK_HARDWARE_TYPE=1)
  elseif(CMAKE_SIZEOF_VOID_P EQUAL 8)
     set(ARCH armv8)
     add_definitions(-DDJIOSDK_HARDWARE_TYPE=2)
  endif()
elseif()
  add_definitions(-DDJIOSDK_HARDWARE_TYPE=0)
endif()

add_definitions(-DADVANCED_SENSING)

include(FetchContent)
FetchContent_Declare(
dji_onboardsdk
GIT_REPOSITORY https://github.com/aerostack2/Onboard-SDK.git
GIT_TAG master
)
FetchContent_MakeAvailable(dji_onboardsdk)


set(ONBOARDSDK_SOURCE ${dji_onboardsdk_SOURCE_DIR}/osdk-core)
message(STATUS "ONBOARDSDK_SOURCE: ${ONBOARDSDK_SOURCE}")

set(SOURCE_CPP_FILES 
  src/dji_matrice_platform.cpp
  src/dji_camera_handler.cpp
  src/dji_mop_handler.cpp
)
set(HEADER_HPP_FILES
  include/dji_matrice_platform.hpp
)

message(STATUS : "FOLDER: ${ONBOARDSDK_SOURCE}/utility/inc")
set(CURRENT_CMAKE_MODULE_PATH ${ONBOARDSDK_SOURCE}/cmake-modules)
    
include_directories(
  include
  include/${PROJECT_NAME}
  ${ONBOARDSDK_SOURCE}/../sample/platform/linux/common
  ${ONBOARDSDK_SOURCE}/../sample/platform/linux/hal
  ${ONBOARDSDK_SOURCE}/../sample/platform/linux/osal
  ${ONBOARDSDK_SOURCE}/../sample/core/inc
  ${ONBOARDSDK_SOURCE}/api/inc
  ${ONBOARDSDK_SOURCE}/logger/inc
  ${ONBOARDSDK_SOURCE}/platform/inc
  ${ONBOARDSDK_SOURCE}/utility/inc
  ${ONBOARDSDK_SOURCE}/hal/inc
  ${ONBOARDSDK_SOURCE}/modules/inc/payload
  ${ONBOARDSDK_SOURCE}/modules/inc/filemgr
  ${ONBOARDSDK_SOURCE}/modules/inc/firewall
  ${ONBOARDSDK_SOURCE}/modules/inc/flight
  ${ONBOARDSDK_SOURCE}/modules/inc/mop
  ${ONBOARDSDK_SOURCE}/protocol/inc
  ${ONBOARDSDK_SOURCE}/platform/linux/inc
  ${ONBOARDSDK_SOURCE}/advanced-sensing/api/inc
  ${ONBOARDSDK_SOURCE}/linker/${ARCH}/inc
  ${ONBOARDSDK_SOURCE}/../sample/platform/linux/hal
  ${ONBOARDSDK_SOURCE}/../sample/platform/linux/hal/hotplug
  ${ONBOARDSDK_SOURCE}/../sample/platform/linux/osal
  ${ONBOARDSDK_SOURCE}/../sample/core/inc
  ${ONBOARDSDK_SOURCE}/../sample/platform/linux/common
)

FILE(GLOB SOURCE_FILES *.hpp *.cpp
    ${ONBOARDSDK_SOURCE}/../sample/core/src/flight_sample.cpp
    ${ONBOARDSDK_SOURCE}/../sample/platform/linux/common/dji_linux_environment.cpp
    ${ONBOARDSDK_SOURCE}/../sample/platform/linux/common/dji_linux_helpers.cpp
    ${ONBOARDSDK_SOURCE}/../sample/platform/linux/hal/osdkhal_linux.c
    ${ONBOARDSDK_SOURCE}/../sample/platform/linux/osal/osdkosal_linux.c
    )

find_package(OpenCV REQUIRED)

add_executable(${PROJECT_NAME}_node src/dji_matrice_platform_main.cpp ${SOURCE_CPP_FILES} ${SOURCE_FILES} )
target_link_libraries(${PROJECT_NAME}_node djiosdk-core ${OpenCV_LIBS})
ament_target_dependencies(${PROJECT_NAME}_node ${PROJECT_DEPENDENCIES})

if(BUILD_TESTING)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)

  # find_package(ament_lint_auto REQUIRED)
  # ament_lint_auto_find_test_dependencies()
  
  ament_cppcheck(src/ include/)
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
  DESTINATION lib/${PROJECT_NAME})

ament_package()
