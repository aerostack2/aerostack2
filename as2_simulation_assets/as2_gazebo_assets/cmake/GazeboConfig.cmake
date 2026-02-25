# GazeboConfig.cmake
# Auto-detects Gazebo version and includes appropriate configuration

# Default to Fortress
set(GZ_VERSION "fortress")

# Check if Harmonic libs are available
find_package(gz-common5 QUIET)
if(gz-common5_FOUND)
  set(GZ_VERSION "harmonic")
endif()

message(STATUS "Detected Gazebo version: ${GZ_VERSION}")

# Include version-specific configuration
if(${GZ_VERSION} STREQUAL "harmonic")
  include(${CMAKE_CURRENT_LIST_DIR}/GazeboHarmonic.cmake)
elseif(${GZ_VERSION} STREQUAL "fortress")
  include(${CMAKE_CURRENT_LIST_DIR}/GazeboFortress.cmake)
else()
  message(FATAL_ERROR "Unsupported Gazebo version: ${GZ_VERSION}")
endif()

# Find all required Gazebo packages
find_package(${GZ_SIM} REQUIRED)
find_package(${GZ_COMMON} REQUIRED COMPONENTS graphics)
find_package(${GZ_MATH} REQUIRED)
find_package(${GZ_TRANSPORT} REQUIRED)
find_package(${GZ_PLUGIN} REQUIRED COMPONENTS loader register)
find_package(${GZ_RENDERING} REQUIRED)
find_package(${GZ_MSGS} REQUIRED)
