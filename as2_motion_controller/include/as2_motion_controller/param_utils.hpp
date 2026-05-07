// Copyright 2023 Universidad Politécnica de Madrid
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
//    * Neither the name of the Universidad Politécnica de Madrid nor the names
//    of its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
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

/*!*******************************************************************************************
 *  @file       param_utils.hpp
 *  @brief      Generic parameter utilities shared by all as2_motion_controller plugins.
 *  @authors    Rafael Pérez Seguí
 ********************************************************************************************/

#ifndef AS2_MOTION_CONTROLLER__PARAM_UTILS_HPP_
#define AS2_MOTION_CONTROLLER__PARAM_UTILS_HPP_

#include <Eigen/Dense>  // NOLINT(build/include_order)
#include <cstddef>
#include <cstdint>
#include <algorithm>
#include <array>
#include <string>
#include <vector>
#include <rclcpp/exceptions.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/parameter.hpp>

namespace as2_motion_controller_param_utils
{

/**
 * @brief Read a scalar parameter of type T from the node.
 *
 * Specializations are provided for bool, int64_t, double and std::string.
 * Throws rclcpp::exceptions::ParameterNotDeclaredException if the parameter
 * was not declared, or rclcpp::ParameterTypeException on type mismatch.
 *
 * @param node Pointer to the ROS 2 node holding the parameters.
 * @param name Fully-qualified parameter name.
 * @return Parameter value of type T.
 */
template<typename T>
T readParam(rclcpp::Node * node, const std::string & name);

template<>
inline bool readParam<bool>(rclcpp::Node * node, const std::string & name)
{
  return node->get_parameter(name).as_bool();
}

template<>
inline int64_t readParam<int64_t>(rclcpp::Node * node, const std::string & name)
{
  return node->get_parameter(name).as_int();
}

template<>
inline double readParam<double>(rclcpp::Node * node, const std::string & name)
{
  return node->get_parameter(name).as_double();
}

template<>
inline std::string readParam<std::string>(rclcpp::Node * node, const std::string & name)
{
  return node->get_parameter(name).as_string();
}

/**
 * @brief Read a fixed-size double array parameter into std::array<double, N>.
 *
 * The runtime size is checked against the compile-time N. A size mismatch is
 * treated as fatal: the function logs RCLCPP_FATAL and throws
 * rclcpp::exceptions::InvalidParameterValueException so the controller does
 * not silently run with a half-configured solver.
 *
 * @tparam N Expected number of elements in the array.
 * @param node Pointer to the ROS 2 node.
 * @param name Fully-qualified parameter name.
 * @return Fixed-size std::array<double, N> with the values.
 */
template<std::size_t N>
std::array<double, N> readArray(rclcpp::Node * node, const std::string & name)
{
  const auto values = node->get_parameter(name).as_double_array();
  if (values.size() != N) {
    RCLCPP_FATAL(
      node->get_logger(),
      "Parameter '%s' has size %zu, expected %zu",
      name.c_str(), values.size(), N);
    throw rclcpp::exceptions::InvalidParameterValueException(
            "Parameter '" + name + "' has wrong size");
  }
  std::array<double, N> out{};
  std::copy_n(values.begin(), N, out.begin());
  return out;
}

/**
 * @brief Read a variable-size double array parameter.
 *
 * If expected_size != 0, the size is validated and a mismatch is fatal
 * (RCLCPP_FATAL + throw). When expected_size == 0, any size is accepted.
 *
 * @param node Pointer to the ROS 2 node.
 * @param name Fully-qualified parameter name.
 * @param expected_size Expected number of elements, or 0 to skip the check.
 * @return Vector with the parameter values.
 */
std::vector<double> readDoubleArray(
  rclcpp::Node * node,
  const std::string & name,
  std::size_t expected_size = 0);

/**
 * @brief Read a 3-component double array parameter as Eigen::Vector3d.
 * @param node Pointer to the ROS 2 node.
 * @param name Fully-qualified parameter name.
 * @return Eigen::Vector3d with the values.
 */
inline Eigen::Vector3d readVector3(rclcpp::Node * node, const std::string & name)
{
  const auto a = readArray<3>(node, name);
  return Eigen::Vector3d(a[0], a[1], a[2]);
}

/**
 * @brief True if every element of values is NaN.
 *
 * Used as a sentinel for "intentionally empty" optional double-array
 * parameters (e.g. unconstrained limits in MPC plugins). The YAML keeps a
 * non-empty array because ROS 2 forbids empty array overrides.
 *
 * @param values Array values to test.
 * @return true if values is non-empty and every element is NaN.
 */
bool isNanSentinel(const std::vector<double> & values);

}  // namespace as2_motion_controller_param_utils

#endif  // AS2_MOTION_CONTROLLER__PARAM_UTILS_HPP_
