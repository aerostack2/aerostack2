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
 * @file frame_transform.cpp
 * @addtogroup lib
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 *
 * Adapted from MAVROS ftf_frame_conversions.cpp and ftf_quaternion_utils.cpp.
 */

#include <px4_ros_com/frame_transforms.h>

#include <assert.h>

namespace px4_ros_com
{
namespace frame_transforms
{

// Utils to ease conversions
namespace utils
{

// Quaternion
namespace quaternion
{

Eigen::Quaterniond quaternion_from_euler(const Eigen::Vector3d &euler)
{
	// YPR is ZYX axes
	return Eigen::Quaterniond(Eigen::AngleAxisd(euler.z(), Eigen::Vector3d::UnitZ()) *
				  Eigen::AngleAxisd(euler.y(), Eigen::Vector3d::UnitY()) *
				  Eigen::AngleAxisd(euler.x(), Eigen::Vector3d::UnitX()));
}

Eigen::Quaterniond quaternion_from_euler(const double roll, const double pitch, const double yaw)
{
	return quaternion_from_euler(Eigen::Vector3d(roll, pitch, yaw));
}

Eigen::Vector3d quaternion_to_euler(const Eigen::Quaterniond &q)
{
	// YPR is ZYX axes
	return q.toRotationMatrix().eulerAngles(2, 1, 0).reverse();
}

void quaternion_to_euler(const Eigen::Quaterniond &q, double &roll, double &pitch, double &yaw)
{
	const auto euler = quaternion_to_euler(q);
	roll = euler.x();
	pitch = euler.y();
	yaw = euler.z();
}

void eigen_quat_to_array(const Eigen::Quaterniond &q, std::array<float, 4> &qarray)
{
	qarray[0] = q.w();
	qarray[1] = q.x();
	qarray[2] = q.y();
	qarray[3] = q.z();
}

Eigen::Quaterniond array_to_eigen_quat(const std::array<float, 4> &q)
{
	return Eigen::Quaterniond(q[0], q[1], q[2], q[3]);
}

double quaternion_get_yaw(const Eigen::Quaterniond &q)
{
	const double &q0 = q.w();
	const double &q1 = q.x();
	const double &q2 = q.y();
	const double &q3 = q.z();

	return std::atan2(2. * (q0 * q3 + q1 * q2), 1. - 2. * (q2 * q2 + q3 * q3));
}

} // namespace quaternion

// Data types
namespace types
{

template <class T, std::size_t SIZE> void covariance_to_array(const T &cov, std::array<float, SIZE> &covmsg)
{
	std::copy(cov.cbegin(), cov.cend(), covmsg.begin());
}

template <class T, std::size_t ARR_SIZE>
void covariance_urt_to_array(const T &covmap, std::array<float, ARR_SIZE> &covmsg)
{
	auto m = covmap;
	std::size_t COV_SIZE = m.rows() * (m.rows() + 1) / 2;
	assert(COV_SIZE == ARR_SIZE &&
	       ("covariance matrix URT size (%lu) is different from uORB msg covariance field size (%lu)", COV_SIZE,
		ARR_SIZE));

	auto out = covmsg.begin();

	for (size_t x = 0; x < m.cols(); x++) {
		for (size_t y = x; y < m.rows(); y++)
			*out++ = m(y, x);
	}
}

template <class T, std::size_t ARR_SIZE>
void array_urt_to_covariance_matrix(const std::array<float, ARR_SIZE> &covmsg, T &covmat)
{
	std::size_t COV_SIZE = covmat.rows() * (covmat.rows() + 1) / 2;
	assert(COV_SIZE == ARR_SIZE &&
	       ("covariance matrix URT size (%lu) is different from uORB msg covariance field size (%lu)", COV_SIZE,
		ARR_SIZE));

	auto in = covmsg.begin();

	for (size_t x = 0; x < covmat.cols(); x++) {
		for (size_t y = x; y < covmat.rows(); y++) {
			covmat(x, y) = static_cast<double>(*in++);
			covmat(y, x) = covmat(x, y);
		}
	}
}

} // namespace types
} // namespace utils

Eigen::Quaterniond transform_orientation(const Eigen::Quaterniond &q, const StaticTF transform)
{
	Eigen::Quaterniond out;

	switch (transform) {
	case StaticTF::NED_TO_ENU:
	case StaticTF::ENU_TO_NED:
		out = NED_ENU_Q * q;
		break;

	case StaticTF::AIRCRAFT_TO_BASELINK:
	case StaticTF::BASELINK_TO_AIRCRAFT:
		out = q * AIRCRAFT_BASELINK_Q;
		break;

	default:
		break;
	}

	return out;
}

Eigen::Vector3d transform_static_frame(const Eigen::Vector3d &vec, const StaticTF transform)
{
	Eigen::Vector3d out;
	switch (transform) {
	case StaticTF::NED_TO_ENU:
	case StaticTF::ENU_TO_NED:
		out = NED_ENU_REFLECTION_XY * (NED_ENU_REFLECTION_Z * vec);
		break;

	case StaticTF::AIRCRAFT_TO_BASELINK:
	case StaticTF::BASELINK_TO_AIRCRAFT:
		out = AIRCRAFT_BASELINK_AFFINE * vec;
		break;

	default:
		break;
	}

	return out;
}

Covariance3d transform_static_frame(const Covariance3d &cov, const StaticTF transform)
{
	Covariance3d cov_out_;
	EigenMapConstCovariance3d cov_in(cov.data());
	EigenMapCovariance3d cov_out(cov_out_.data());

	switch (transform) {
	case StaticTF::NED_TO_ENU:
	case StaticTF::ENU_TO_NED:
		cov_out = NED_ENU_REFLECTION_XY * (NED_ENU_REFLECTION_Z * cov_in * NED_ENU_REFLECTION_Z) *
			  NED_ENU_REFLECTION_XY.transpose();
		break;

	case StaticTF::AIRCRAFT_TO_BASELINK:
	case StaticTF::BASELINK_TO_AIRCRAFT:
		cov_out = cov_in * AIRCRAFT_BASELINK_Q;
		break;

	default:
		break;
	}

	return cov_out_;
}

Covariance6d transform_static_frame(const Covariance6d &cov, const StaticTF transform)
{
	Covariance6d cov_out_;
	Matrix6d R = Matrix6d::Zero(); // not `auto` because Zero ret is const

	EigenMapConstCovariance6d cov_in(cov.data());
	EigenMapCovariance6d cov_out(cov_out_.data());

	switch (transform) {
	case StaticTF::NED_TO_ENU:
	case StaticTF::ENU_TO_NED: {
		Eigen::PermutationMatrix<6> NED_ENU_REFLECTION_XY_6(NED_ENU_REFLECTION_XY.indices().replicate<2, 1>());
		NED_ENU_REFLECTION_XY_6.indices().middleRows<3>(3).array() += 3;
		Eigen::DiagonalMatrix<double, 6> NED_ENU_REFLECTION_Z_6(
		    NED_ENU_REFLECTION_Z.diagonal().replicate<2, 1>());

		cov_out = NED_ENU_REFLECTION_XY_6 * (NED_ENU_REFLECTION_Z_6 * cov_in * NED_ENU_REFLECTION_Z_6) *
			  NED_ENU_REFLECTION_XY_6.transpose();
		break;
	}
	case StaticTF::AIRCRAFT_TO_BASELINK:
	case StaticTF::BASELINK_TO_AIRCRAFT:
		R.block<3, 3>(0, 0) =
			R.block<3, 3>(3, 3) =
		AIRCRAFT_BASELINK_R;

		cov_out = R * cov_in * R.transpose();
		break;

	default:
		break;
	}

	return cov_out_;
}

Covariance9d transform_static_frame(const Covariance9d &cov, const StaticTF transform)
{
	Covariance9d cov_out_;
	Matrix9d R = Matrix9d::Zero();

	EigenMapConstCovariance9d cov_in(cov.data());
	EigenMapCovariance9d cov_out(cov_out_.data());

	switch (transform) {
	case StaticTF::NED_TO_ENU:
	case StaticTF::ENU_TO_NED: {
		Eigen::PermutationMatrix<9> NED_ENU_REFLECTION_XY_9(NED_ENU_REFLECTION_XY.indices().replicate<3, 1>());
		NED_ENU_REFLECTION_XY_9.indices().middleRows<3>(3).array() += 3;
		NED_ENU_REFLECTION_XY_9.indices().middleRows<3>(6).array() += 6;
		Eigen::DiagonalMatrix<double, 9> NED_ENU_REFLECTION_Z_9(
		    NED_ENU_REFLECTION_Z.diagonal().replicate<3, 1>());

		cov_out = NED_ENU_REFLECTION_XY_9 * (NED_ENU_REFLECTION_Z_9 * cov_in * NED_ENU_REFLECTION_Z_9) *
			  NED_ENU_REFLECTION_XY_9.transpose();

		break;
	}
	case StaticTF::AIRCRAFT_TO_BASELINK:
	case StaticTF::BASELINK_TO_AIRCRAFT:
		R.block<3, 3>(0, 0) =
			R.block<3, 3>(3, 3) =
				R.block<3, 3>(6, 6) =
		AIRCRAFT_BASELINK_R;

		cov_out = R * cov_in * R.transpose();

		break;

	default:
		break;
	}

	return cov_out_;
}

Eigen::Vector3d transform_static_frame(const Eigen::Vector3d &vec, const Eigen::Vector3d &map_origin,
				       const StaticTF transform)
{
	//! Degrees to radians
	static constexpr double DEG_TO_RAD = (M_PI / 180.0);

	// Don't forget to convert from degrees to radians
	const double sin_lat = std::sin(map_origin.x() * DEG_TO_RAD);
	const double sin_lon = std::sin(map_origin.y() * DEG_TO_RAD);
	const double cos_lat = std::cos(map_origin.x() * DEG_TO_RAD);
	const double cos_lon = std::cos(map_origin.y() * DEG_TO_RAD);

	/**
	 * @brief Compute transform from ECEF to ENU:
	 * http://www.navipedia.net/index.php/Transformations_between_ECEF_and_ENU_coordinates
	 * ϕ = latitude
	 * λ = longitude
	 * The rotation is composed by a counter-clockwise rotation over the Z-axis
	 * by an angle of 90 + λ followed by a counter-clockwise rotation over the east-axis by
	 * an angle of 90 - ϕ.
	 * R = [-sinλ         cosλ         0.0
	 *      -cosλ*sinϕ   -sinλ*sinϕ    cosϕ
	 *       cosλ*cosϕ    sinλ*cosϕ    sinϕ   ]
	 * [East, North, Up] = R * [∂x, ∂y, ∂z]
	 * where both [East, North, Up] and [∂x, ∂y, ∂z] are local coordinates relative to map origin.
	 */
	Eigen::Matrix3d R;
	R << -sin_lon,            cos_lon,           0.0,
            -cos_lon * sin_lat, -sin_lon * sin_lat, cos_lat,
             cos_lon * cos_lat,  sin_lon * cos_lat, sin_lat;


	Eigen::Vector3d out;
	switch (transform) {
	case StaticTF::ECEF_TO_ENU:
		out = R * vec;
		break;

	case StaticTF::ENU_TO_ECEF:
		// ENU to ECEF rotation is just an inverse rotation from ECEF to ENU, which means transpose.
		R.transposeInPlace();
		out = R * vec;
		break;

	default:
		break;
	}

	return out;
}

Eigen::Vector3d transform_frame(const Eigen::Vector3d &vec, const Eigen::Quaterniond &q)
{
	Eigen::Affine3d transformation(q);
	return transformation * vec;
}

Covariance3d transform_frame(const Covariance3d &cov, const Eigen::Quaterniond &q)
{
	Covariance3d cov_out_;
	EigenMapConstCovariance3d cov_in(cov.data());
	EigenMapCovariance3d cov_out(cov_out_.data());

	cov_out = cov_in * q;
	return cov_out_;
}

Covariance6d transform_frame(const Covariance6d &cov, const Eigen::Quaterniond &q)
{
	Covariance6d cov_out_;
	Matrix6d R = Matrix6d::Zero();

	EigenMapConstCovariance6d cov_in(cov.data());
	EigenMapCovariance6d cov_out(cov_out_.data());

	R.block<3, 3>(0, 0) =
		R.block<3, 3>(3, 3) =
	q.normalized().toRotationMatrix();

	cov_out = R * cov_in * R.transpose();
	return cov_out_;
}

Covariance9d transform_frame(const Covariance9d &cov, const Eigen::Quaterniond &q)
{
	Covariance9d cov_out_;
	Matrix9d R = Matrix9d::Zero();

	EigenMapConstCovariance9d cov_in(cov.data());
	EigenMapCovariance9d cov_out(cov_out_.data());

	R.block<3, 3>(0, 0) =
		R.block<3, 3>(3, 3) =
			R.block<3, 3>(6, 6) =
	q.normalized().toRotationMatrix();

	cov_out = R * cov_in * R.transpose();
	return cov_out_;
}

} // namespace frame_transforms
} // namespace px4_ros_com
