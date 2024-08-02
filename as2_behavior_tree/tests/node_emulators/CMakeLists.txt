include_directories(
  include
  include/${PROJECT_NAME}
)

add_executable(platform_emulator src/platform_emulator.cpp)
target_include_directories(platform_emulator
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(platform_emulator ${PROJECT_DEPENDENCIES})

add_executable(takeoff_emulator src/takeoff_emulator.cpp)
target_include_directories(takeoff_emulator
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(takeoff_emulator ${PROJECT_DEPENDENCIES})

add_executable(land_emulator src/land_emulator.cpp)
target_include_directories(land_emulator
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(land_emulator ${PROJECT_DEPENDENCIES})

add_executable(go_to_emulator src/go_to_emulator.cpp)
target_include_directories(go_to_emulator
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(go_to_emulator ${PROJECT_DEPENDENCIES})

add_executable(follow_path_emulator src/follow_path_emulator.cpp)
target_include_directories(follow_path_emulator
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(follow_path_emulator ${PROJECT_DEPENDENCIES})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

install(TARGETS platform_emulator
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS takeoff_emulator
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS land_emulator
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS go_to_emulator
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS follow_path_emulator
DESTINATION lib/${PROJECT_NAME})
