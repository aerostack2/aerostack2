file(GLOB EXAMPLES_SOURCE "*_example.cpp")

# Print all examples
foreach(EXAMPLE_FILE ${EXAMPLES_SOURCE})
  get_filename_component(EXAMPLE_NAME ${EXAMPLE_FILE} NAME_WE)
  message(STATUS "Example: ${EXAMPLE_NAME}")
endforeach()

foreach(EXAMPLE_FILE ${EXAMPLES_SOURCE})
  get_filename_component(EXAMPLE_NAME ${EXAMPLE_FILE} NAME_WE)

  # Create an executable for each example
  add_executable(${PROJECT_NAME}_${EXAMPLE_NAME} ${EXAMPLE_FILE})
  target_link_libraries(${PROJECT_NAME}_${EXAMPLE_NAME} ${PROJECT_NAME})
endforeach()
