# Ensures yaml-cpp::yaml-cpp target exists across different yaml-cpp installations
if(NOT TARGET yaml-cpp::yaml-cpp)
  if(TARGET yaml-cpp)
    add_library(yaml-cpp::yaml-cpp ALIAS yaml-cpp)
    message(STATUS "Created yaml-cpp::yaml-cpp alias from yaml-cpp target")
  elseif(YAML_CPP_LIBRARIES)
    # Legacy installations only set variables
    add_library(yaml-cpp::yaml-cpp INTERFACE IMPORTED)
    set_target_properties(yaml-cpp::yaml-cpp PROPERTIES
      INTERFACE_LINK_LIBRARIES "${YAML_CPP_LIBRARIES}"
      INTERFACE_INCLUDE_DIRECTORIES "${YAML_CPP_INCLUDE_DIRS}"
    )
    message(STATUS "Created yaml-cpp::yaml-cpp target from legacy variables")
  else()
    message(FATAL_ERROR "yaml-cpp not found or incompatible. Ensure find_package(yaml-cpp) is called before including this module.")
  endif()
endif()
