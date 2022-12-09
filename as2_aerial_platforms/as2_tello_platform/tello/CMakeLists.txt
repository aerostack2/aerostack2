cmake_minimum_required(VERSION 3.13)
set(PROJECT_NAME tello)
project(${PROJECT_NAME})

set(CMAKE_CXX_STANDARD 17)

include_directories(
    include
    include/${PROJECT_NAME}
)

find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

set(CPP_FILES
  src/socket_udp.cpp
  src/tello.cpp
)

# LIB
add_library(${PROJECT_NAME} ${CPP_FILES})
target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
    ${CMAKE_THREAD_LIBS_INIT}
    # pthread
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

# TEST
add_executable(tello_test test/tello_test.cpp)
target_link_libraries(tello_test
    ${PROJECT_NAME}
    ${OpenCV_LIBS}
)
