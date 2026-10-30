// Copyright 2026 Universidad Politécnica de Madrid
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/**
 * @file gz_compat.hpp
 *
 * Lets plugins be written once against Gazebo Harmonic and still build on
 * Fortress.
 *
 * Fortress already ships the gz/... headers and the gz namespace, but not
 * the gz-prefixed macros or a few newer method names. This header fills
 * those gaps. On Harmonic it adds nothing.
 *
 * <gz/plugin/Register.hh> must only be included by the one source file that
 * registers the plugin, so it is not included here. Include this header
 * after it to get GZ_ADD_PLUGIN on Fortress.
 */

#ifndef AS2_SIMULATION_ASSETS__AS2_GAZEBO_ASSETS__PLUGINS__GZ_COMPAT_HPP_
#define AS2_SIMULATION_ASSETS__AS2_GAZEBO_ASSETS__PLUGINS__GZ_COMPAT_HPP_

#include <gz/common/Console.hh>
#include <gz/common/Profiler.hh>
#include <gz/math/config.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>

#ifndef gzerr
#define gzerr ignerr
#endif
#ifndef gzwarn
#define gzwarn ignwarn
#endif
#ifndef gzmsg
#define gzmsg ignmsg
#endif
#ifndef gzdbg
#define gzdbg igndbg
#endif

#ifndef GZ_PROFILE
#define GZ_PROFILE IGN_PROFILE
#endif

#if !defined(GZ_ADD_PLUGIN) && defined(IGNITION_ADD_PLUGIN)
#define GZ_ADD_PLUGIN IGNITION_ADD_PLUGIN
#endif
#if !defined(GZ_ADD_PLUGIN_ALIAS) && defined(IGNITION_ADD_PLUGIN_ALIAS)
#define GZ_ADD_PLUGIN_ALIAS IGNITION_ADD_PLUGIN_ALIAS
#endif

namespace gz_compat
{

/// Quaternion::From2Axes was renamed to SetFrom2Axes in gz-math 7.
template<typename T>
void setFrom2Axes(
  gz::math::Quaternion<T> & q, const gz::math::Vector3<T> & v1,
  const gz::math::Vector3<T> & v2)
{
#if defined(GZ_MATH_MAJOR_VERSION) && GZ_MATH_MAJOR_VERSION >= 7
  q.SetFrom2Axes(v1, v2);
#else
  q.From2Axes(v1, v2);
#endif
}

}  // namespace gz_compat

#endif  // AS2_SIMULATION_ASSETS__AS2_GAZEBO_ASSETS__PLUGINS__GZ_COMPAT_HPP_
