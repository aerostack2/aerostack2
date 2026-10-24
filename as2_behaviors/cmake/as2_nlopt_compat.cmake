# Keep the legacy NLopt FetchContent project usable with CMake 4.x.
#
# NLopt 2.7.1 declares CMake versions older than the compatibility floor in
# CMake 4.4.  The policy minimum is inherited by nested FetchContent projects;
# the code-generation scripts are patched after FetchContent has populated the
# source tree because they are later invoked with `cmake -P` and do not inherit
# the cache variable themselves.

set(CMAKE_POLICY_VERSION_MINIMUM "3.5" CACHE STRING
  "Minimum policy version for legacy FetchContent projects" FORCE)

function(as2_patch_nlopt_cmake_compat)
  set(_as2_nlopt_source_dir "${CMAKE_BINARY_DIR}/_deps/nlopt-src")

  foreach(_as2_nlopt_script IN ITEMS generate-cpp.cmake generate-fortran.cmake)
    set(_as2_nlopt_script_path "${_as2_nlopt_source_dir}/cmake/${_as2_nlopt_script}")
    if(EXISTS "${_as2_nlopt_script_path}")
      file(READ "${_as2_nlopt_script_path}" _as2_nlopt_contents)
      string(FIND "${_as2_nlopt_contents}" "cmake_minimum_required(VERSION 3.5)\n"
        _as2_nlopt_prefix)
      if(_as2_nlopt_prefix EQUAL -1)
        string(FIND "${_as2_nlopt_contents}" "\n" _as2_nlopt_newline)
        if(_as2_nlopt_newline GREATER -1)
          math(EXPR _as2_nlopt_body_start "${_as2_nlopt_newline} + 1")
          string(SUBSTRING "${_as2_nlopt_contents}" "${_as2_nlopt_body_start}" -1
            _as2_nlopt_body)
          set(_as2_nlopt_contents "cmake_minimum_required(VERSION 3.5)\n${_as2_nlopt_body}")
        else()
          set(_as2_nlopt_contents "cmake_minimum_required(VERSION 3.5)\n")
        endif()
        file(WRITE "${_as2_nlopt_script_path}" "${_as2_nlopt_contents}")
      endif()
    endif()
  endforeach()
endfunction()
