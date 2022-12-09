#
# Copyright (c) 2017-2018 PX4 Pro Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# 3. Neither the name PX4 nor the names of its contributors may be used to
#    endorse or promote products derived from this software without specific
#    prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

option(SANITIZE_ADDRESS "Enable AddressSanitizer" OFF)
option(SANITIZE_MEMORY "Enable MemorySanitizer" OFF)
option(SANITIZE_THREAD "Enable ThreadSanitizer" OFF)
option(SANITIZE_UNDEFINED "Enable UndefinedBehaviorSanitizer" OFF)

set(SANITIZERS)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 4.7)
    set(SANITIZERS address thread)
  else()
    message(
      SEND_ERROR "Cannot use address and thread sanitizers without gcc >= 4.7")
  endif()
  if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 4.9)
    set(SANITIZERS ${SANITIZERS} undefined)
  else()
    message(SEND_ERROR "Cannot use undefined sanitizers without gcc >= 4.9")
  endif()
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 4.1)
    set(SANITIZERS address memory thread undefined)
  else()
    message(SEND_ERROR "Cannot use sanitizers without clang >= 4.1")
  endif()
endif()

add_compile_options(-g3)

if(SANITIZE_ADDRESS AND (address IN_LIST SANITIZERS))
  add_compile_options(-fno-omit-frame-pointer -fno-optimize-sibling-calls
                      -fsanitize=address)
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fsanitize=address")
  set(CMAKE_SHARED_LINKER_FLAGS
      "${CMAKE_SHARED_LINKER_FLAGS} -fsanitize=address")
  set(CMAKE_MODULE_LINKER_FLAGS
      "${CMAKE_MODULE_LINKER_FLAGS} -fsanitize=address")

  message(
    STATUS
      "AddressSanitizer enabled for ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION} compiler"
    )

endif()

if(SANITIZE_MEMORY AND (memory IN_LIST SANITIZERS))
  add_compile_options(-fsanitize=memory)
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fsanitize=memory")
  set(CMAKE_SHARED_LINKER_FLAGS
      "${CMAKE_SHARED_LINKER_FLAGS} -fsanitize=memory")
  set(CMAKE_MODULE_LINKER_FLAGS
      "${CMAKE_MODULE_LINKER_FLAGS} -fsanitize=memory")

  message(
    STATUS
      "MemorySanitizer enabled for ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION} compiler"
    )

endif()

# AddressSanitizer and ThreadSanitizer are not compatible together
if(SANITIZE_THREAD AND (NOT SANITIZE_ADDRESS) AND (thread IN_LIST SANITIZERS))
  add_compile_options(-fsanitize=thread)
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fsanitize=thread")
  set(CMAKE_SHARED_LINKER_FLAGS
      "${CMAKE_SHARED_LINKER_FLAGS} -fsanitize=thread")
  set(CMAKE_MODULE_LINKER_FLAGS
      "${CMAKE_MODULE_LINKER_FLAGS} -fsanitize=thread")

  message(
    STATUS
      "ThreadSanitizer enabled for ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION} compiler"
    )

endif()

if(SANITIZE_UNDEFINED AND (undefined IN_LIST SANITIZERS))
  add_compile_options(-fsanitize=alignment
                      -fsanitize=bool
                      -fsanitize=bounds
                      -fsanitize=enum
                      -fsanitize=float-cast-overflow
                      -fsanitize=float-divide-by-zero
                      -fsanitize=integer-divide-by-zero
                      -fsanitize=nonnull-attribute
                      -fsanitize=null
                      -fsanitize=object-size
                      -fsanitize=return
                      -fsanitize=returns-nonnull-attribute
                      -fsanitize=shift
                      -fsanitize=signed-integer-overflow
                      -fsanitize=unreachable
                      -fsanitize=vla-bound
                      -fsanitize=vptr
                      -fno-sanitize-recover=bounds,null)
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fsanitize=undefined")
  set(CMAKE_SHARED_LINKER_FLAGS
      "${CMAKE_SHARED_LINKER_FLAGS} -fsanitize=undefined")
  set(CMAKE_MODULE_LINKER_FLAGS
      "${CMAKE_MODULE_LINKER_FLAGS} -fsanitize=undefined")

  message(
    STATUS
      "UndefinedBehaviorSanitizer enabled for ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION} compiler"
    )

endif()
