# Copyright 2024 Universidad Politécnica de Madrid
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
# Extra CMake configuration for as2_core
# This file is exported to downstream packages to help them find GeographicLib

# Add GeographicLib CMake module path for Linux systems
if(UNIX AND NOT APPLE)
  if(EXISTS /usr/share/cmake/geographiclib)
    # https://stackoverflow.com/a/73279950/9553849
    list(APPEND CMAKE_MODULE_PATH /usr/share/cmake/geographiclib)
  endif()
endif()

# Ensure GeographicLib is found before checking for targets
if(NOT GeographicLib_FOUND)
  find_package(GeographicLib QUIET REQUIRED)
endif()

# Ensures GeographicLib::GeographicLib target exists across different installations
if(NOT TARGET GeographicLib::GeographicLib)
  if(TARGET GeographicLib)
    add_library(GeographicLib::GeographicLib ALIAS GeographicLib)
    message(STATUS "Created GeographicLib::GeographicLib alias from GeographicLib target")
  elseif(GeographicLib_LIBRARIES)
    # Legacy installations only set variables
    add_library(GeographicLib::GeographicLib INTERFACE IMPORTED)
    set_target_properties(GeographicLib::GeographicLib PROPERTIES
      INTERFACE_LINK_LIBRARIES "${GeographicLib_LIBRARIES}"
      INTERFACE_INCLUDE_DIRECTORIES "${GeographicLib_INCLUDE_DIRS}"
    )
    message(STATUS "Created GeographicLib::GeographicLib target from legacy variables")
  else()
    message(
      FATAL_ERROR
      "GeographicLib not found or incompatible. Ensure find_package(GeographicLib) is called before including this module."
    )
  endif()
endif()
