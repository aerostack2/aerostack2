/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Functions to use on frame transforms
 * @file frame_transform.h
 * @addtogroup lib
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 *
 * Adapted from MAVROS frame_tf.h
 */

#ifndef FRAME_TRANSFORMS_H
#define FRAME_TRANSFORMS_H

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <array>

// for Covariance types
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace px4_ros_com
{
namespace frame_transforms
{

//! Type matching rosmsg for 3x3 covariance matrix
using Covariance3d = sensor_msgs::msg::Imu::_angular_velocity_covariance_type;

//! Type matching rosmsg for 6x6 covariance matrix
using Covariance6d = geometry_msgs::msg::PoseWithCovariance::_covariance_type;

//! Type matching rosmsg for 9x9 covariance matrix
using Covariance9d = std::array<double, 81>;

//! Eigen::Map for Covariance3d
using EigenMapCovariance3d = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>;
using EigenMapConstCovariance3d = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>;

//! Eigen::Map for Covariance6d
using EigenMapCovariance6d = Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>;
using EigenMapConstCovariance6d = Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>;

//! Eigen::Map for Covariance9d
using EigenMapCovariance9d = Eigen::Map<Eigen::Matrix<double, 9, 9, Eigen::RowMajor>>;
using EigenMapConstCovariance9d = Eigen::Map<const Eigen::Matrix<double, 9, 9, Eigen::RowMajor>>;

/**
 * @brief Orientation transform options when applying rotations to data
 */
enum class StaticTF {
	NED_TO_ENU,	           //!< change from expressed WRT NED frame to WRT ENU frame
	ENU_TO_NED,	           //!< change from expressed WRT ENU frame to WRT NED frame
	AIRCRAFT_TO_BASELINK,      //!< change from expressed WRT aircraft frame to WRT to baselink frame
	BASELINK_TO_AIRCRAFT,      //!< change from expressed WRT baselnk to WRT aircraft
	ECEF_TO_ENU,	           //!< change from expressed WRT ECEF frame to WRT ENU frame
	ENU_TO_ECEF	           //!< change from expressed WRT ENU frame to WRT ECEF frame
};

// Utils to ease conversions
namespace utils
{

// Quaternion
namespace quaternion
{

/**
 * @brief Convert euler angles to quaternion.
 */
Eigen::Quaterniond quaternion_from_euler(const Eigen::Vector3d &euler);

/**
 * @brief Convert euler angles to quaternion.
 *
 * @return quaternion, same as @a tf::quaternionFromeuler() but in Eigen format.
 */
Eigen::Quaterniond quaternion_from_euler(const double roll, const double pitch, const double yaw);

/**
 * @brief Convert quaternion to euler angles
 * @return Eigen::Quaterniond
 */
Eigen::Vector3d quaternion_to_euler(const Eigen::Quaterniond &q);

/**
 * @brief Convert quaternion to euler angles
 */
void quaternion_to_euler(const Eigen::Quaterniond &q, double &roll, double &pitch, double &yaw);

/**
 * @brief Store Quaternion to float[4]
 * Eigen::Quaterniond xyzw internal order to PX4 quaternion wxyz.
 */
void eigen_quat_to_array(const Eigen::Quaterniond &q, std::array<float, 4> &qarray);

/**
 * @brief Convert float[4] quaternion to Eigen Quaternion
 */
Eigen::Quaterniond array_to_eigen_quat(const std::array<float, 4> &q);

/**
 * @brief Get Yaw angle from quaternion
 */
double quaternion_get_yaw(const Eigen::Quaterniond &q);

} // namespace quaternion

// Data types
namespace types
{

/**
 * @brief Convert covariance matrix to float[n]
 */
template <class T, std::size_t SIZE> void covariance_to_array(const T &cov, std::array<float, SIZE> &covmsg);

/**
 * @brief Convert upper right triangular of a covariance matrix to float[n] array
 */
template <class T, std::size_t ARR_SIZE>
void covariance_urt_to_array(const T &covmap, std::array<float, ARR_SIZE> &covmsg);

/**
 * @brief Convert float[n] array (upper right triangular of a covariance matrix)
 * to Eigen::MatrixXd<n,n> full covariance matrix
 */
template <class T, std::size_t ARR_SIZE>
void array_urt_to_covariance_matrix(const std::array<float, ARR_SIZE> &covmsg, T &covmat);

} // namespace types
} // namespace utils

/**
 * @brief Static quaternion needed for rotating between ENU and NED frames
 * NED to ENU: +PI/2 rotation about Z (Down) followed by a +PI rotation around X (old North/new East)
 * ENU to NED: +PI/2 rotation about Z (Up) followed by a +PI rotation about X (old East/new North)
 */
static const auto NED_ENU_Q = utils::quaternion::quaternion_from_euler(M_PI, 0.0, M_PI_2);

/**
 * @brief Static quaternion needed for rotating between aircraft and base_link frames
 * +PI rotation around X (Forward) axis transforms from Forward, Right, Down (aircraft)
 * Fto Forward, Left, Up (base_link) frames.
 */
static const auto AIRCRAFT_BASELINK_Q = utils::quaternion::quaternion_from_euler(M_PI, 0.0, 0.0);

/**
 * @brief Static vector needed for rotating between ENU and NED frames
 * +PI rotation around X (North) axis follwed by +PI/2 rotation about Z (Down)
 * gives the ENU frame.  Similarly, a +PI rotation about X (East) followed by
 * a +PI/2 roation about Z (Up) gives the NED frame.
 */
static const Eigen::Affine3d NED_ENU_AFFINE(NED_ENU_Q);

/**
 * @brief Static vector needed for rotating between aircraft and base_link frames
 * +PI rotation around X (Forward) axis transforms from Forward, Right, Down (aircraft)
 * Fto Forward, Left, Up (base_link) frames.
 */
static const Eigen::Affine3d AIRCRAFT_BASELINK_AFFINE(AIRCRAFT_BASELINK_Q);

/**
 * @brief 3-D matrices to fill 6-D rotation matrix applied to change covariance matrices coordinate frames
 */
static const auto NED_ENU_R = NED_ENU_Q.normalized().toRotationMatrix();
static const auto AIRCRAFT_BASELINK_R = AIRCRAFT_BASELINK_Q.normalized().toRotationMatrix();

/**
 * @brief Use reflections instead of rotations for NED <-> ENU transformation
 * to avoid NaN/Inf floating point pollution across different axes
 * since in NED <-> ENU the axes are perfectly aligned.
 */
static const Eigen::PermutationMatrix<3> NED_ENU_REFLECTION_XY(Eigen::Vector3i(1, 0, 2));
static const Eigen::DiagonalMatrix<double, 3> NED_ENU_REFLECTION_Z(1, 1, -1);

/**
 * @brief Auxiliar matrices to Covariance transforms
 */
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix9d = Eigen::Matrix<double, 9, 9>;

/**
 * @brief Transform representation of orientation from 1 frame to another.
 * (e.g. transfrom orientation from representing  from base_link -> NED to
 * representing base_link -> ENU)
 */
Eigen::Quaterniond transform_orientation(const Eigen::Quaterniond &q, const StaticTF transform);

/**
 * @brief Transform data expressed in one frame to another.
 */
Eigen::Vector3d transform_frame(const Eigen::Vector3d &vec, const Eigen::Quaterniond &q);

/**
 * @brief Transform 3x3 convariance expressed in one frame to another.
 */
Covariance3d transform_frame(const Covariance3d &cov, const Eigen::Quaterniond &q);

/**
 * @brief Transform 6x6 convariance expressed in one frame to another.
 */
Covariance6d transform_frame(const Covariance6d &cov, const Eigen::Quaterniond &q);

/**
 * @brief Transform 9x9 convariance expressed in one frame to another.
 */
Covariance9d transform_frame(const Covariance9d &cov, const Eigen::Quaterniond &q);

/**
 * @brief Transform data expressed in one frame to another.
 */
Eigen::Vector3d transform_static_frame(const Eigen::Vector3d &vec, const StaticTF transform);

/**
 * @brief Transform 3d convariance expressed in one frame to another.
 */
Covariance3d transform_static_frame(const Covariance3d &cov, const StaticTF transform);

/**
 * @brief Transform 6d convariance expressed in one frame to another.
 */
Covariance6d transform_static_frame(const Covariance6d &cov, const StaticTF transform);

/**
 * @brief Transform 9d convariance expressed in one frame to another
 */
Covariance9d transform_static_frame(const Covariance9d &cov, const StaticTF transform);

/**
 * @brief Transform data expressed in one frame to another frame with additional
 * map origin parameter.
 */
Eigen::Vector3d transform_static_frame(const Eigen::Vector3d &vec, const Eigen::Vector3d &map_origin,
				       const StaticTF transform);

/**
 * @brief Transform from orientation represented WRT NED frame to orientation
 * represented WRT ENU frame
 */
template <class T> inline T ned_to_enu_orientation(const T &in)
{
	return transform_orientation(in, StaticTF::NED_TO_ENU);
}

/**
 * @brief Transform from orientation represented WRT ENU frame to orientation
 * represented WRT NED frame
 */
template <class T> inline T enu_to_ned_orientation(const T &in)
{
	return transform_orientation(in, StaticTF::ENU_TO_NED);
}

/**
 * @brief Transform from orientation represented WRT aircraft body frame to
 * orientation represented WRT base_link body frame
 */
template <class T> inline T aircraft_to_baselink_orientation(const T &in)
{
	return transform_orientation(in, StaticTF::AIRCRAFT_TO_BASELINK);
}

/**
 * @brief Transform from orientation represented WRT base_link body frame to
 * orientation represented WRT aircraft body frame
 */
template <class T> inline T baselink_to_aircraft_orientation(const T &in)
{
	return transform_orientation(in, StaticTF::BASELINK_TO_AIRCRAFT);
}

/**
 * @brief Transform data expressed in NED to ENU local frame.
 */
template <class T> inline T ned_to_enu_local_frame(const T &in)
{
	return transform_static_frame(in, StaticTF::NED_TO_ENU);
}

/**
 * @brief Transform data expressed in ENU to NED frame.
 */
template <class T> inline T enu_to_ned_local_frame(const T &in)
{
	return transform_static_frame(in, StaticTF::ENU_TO_NED);
}

/**
 * @brief Transform data expressed in aircraft body frame to base_link body frame.
 */
template <class T> inline T aircraft_to_baselink_body_frame(const T &in)
{
	return transform_static_frame(in, StaticTF::AIRCRAFT_TO_BASELINK);
}

/**
 * @brief Transform data expressed in base_link body frame to aircraft body frame.
 */
template <class T> inline T baselink_to_aircraft_body_frame(const T &in)
{
	return transform_static_frame(in, StaticTF::BASELINK_TO_AIRCRAFT);
}

/**
 * @brief Transform data expressed in ECEF frame to ENU frame.
 *
 * @param in          local ECEF coordinates [m]
 * @param map_origin  geodetic origin [lla]
 * @returns local ENU coordinates [m].
 */
template <class T> inline T ecef_to_enu_local_frame(const T &in, const T &map_origin)
{
	return transform_static_frame(in, map_origin, StaticTF::ECEF_TO_ENU);
}

/**
 * @brief Transform data expressed in ENU frame to ECEF frame.
 *
 * @param in          local ENU coordinates [m]
 * @param map_origin  geodetic origin [lla]
 * @returns local ECEF coordinates [m].
 */
template <class T> inline T enu_to_ecef_local_frame(const T &in, const T &map_origin)
{
	return transform_static_frame(in, map_origin, StaticTF::ENU_TO_ECEF);
}

/**
 * @brief Transform data expressed in aircraft frame to NED frame.
 * Assumes quaternion represents rotation from aircraft frame to NED frame.
 */
template <class T> inline T aircraft_to_ned_frame(const T &in, const Eigen::Quaterniond &q)
{
	return transform_frame(in, q);
}

/**
 * @brief Transform data expressed in NED to aircraft frame.
 * Assumes quaternion represents rotation from NED to aircraft frame.
 */
template <class T> inline T ned_to_aircraft_frame(const T &in, const Eigen::Quaterniond &q)
{
	return transform_frame(in, q);
}

/**
 * @brief Transform data expressed in aircraft frame to ENU frame.
 * Assumes quaternion represents rotation from aircraft frame to ENU frame.
 */
template <class T> inline T aircraft_to_enu_frame(const T &in, const Eigen::Quaterniond &q)
{
	return transform_frame(in, q);
}

/**
 * @brief Transform data expressed in ENU to aircraft frame.
 * Assumes quaternion represents rotation from ENU to aircraft frame.
 */
template <class T> inline T enu_to_aircraft_frame(const T &in, const Eigen::Quaterniond &q)
{
	return transform_frame(in, q);
}

/**
 * @brief Transform data expressed in baselink to ENU frame.
 * Assumes quaternion represents rotation from basel_link to ENU frame.
 */
template <class T> inline T baselink_to_enu_frame(const T &in, const Eigen::Quaterniond &q)
{
	return transform_frame(in, q);
}

/**
 * @brief Transform data expressed in ENU to base_link frame.
 * Assumes quaternion represents rotation from ENU to base_link frame.
 */
template <class T> inline T enu_to_baselink_frame(const T &in, const Eigen::Quaterniond &q)
{
	return transform_frame(in, q);
}

} // namespace frame_transforms
} // namespace px4_ros_com

#endif // FRAME_TRANSFORMS_H
