/****************************************************************************
 *
 * Copyright 2019 PX4 Development Team. All rights reserved.
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
 * @brief Vehicle GPS position uORB topic listener example
 * @file vehicle_global_position_listener.cpp
 * @addtogroup examples
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_gps_position.hpp>

/**
 * @brief Vehicle GPS position uORB topic data callback
 */
class VehicleGpsPositionListener : public rclcpp::Node
{
public:
	explicit VehicleGpsPositionListener() : Node("vehicle_global_position_listener") {
		subscription_ = this->create_subscription<px4_msgs::msg::VehicleGpsPosition>(
			"fmu/vehicle_gps_position/out",
#ifdef ROS_DEFAULT_API
            10,
#endif
			[this](const px4_msgs::msg::VehicleGpsPosition::UniquePtr msg) {
			std::cout << "\n\n\n\n\n\n\n\n\n\n";
			std::cout << "RECEIVED VEHICLE GPS POSITION DATA"   << std::endl;
			std::cout << "=================================="   << std::endl;
			std::cout << "ts: "      << msg->timestamp    << std::endl;
			std::cout << "lat: " << msg->lat  << std::endl;
			std::cout << "lon: " << msg->lon << std::endl;
			std::cout << "alt: " << msg->alt  << std::endl;
			std::cout << "alt_ellipsoid: " << msg->alt_ellipsoid << std::endl;
			std::cout << "s_variance_m_s: " << msg->s_variance_m_s << std::endl;
			std::cout << "c_variance_rad: " << msg->c_variance_rad << std::endl;
			std::cout << "fix_type: " << msg->fix_type << std::endl;
			std::cout << "eph: " << msg->eph << std::endl;
			std::cout << "epv: " << msg->epv << std::endl;
			std::cout << "hdop: " << msg->hdop << std::endl;
			std::cout << "vdop: " << msg->vdop << std::endl;
			std::cout << "noise_per_ms: " << msg->noise_per_ms << std::endl;
			std::cout << "vel_m_s: " << msg->vel_m_s << std::endl;
			std::cout << "vel_n_m_s: " << msg->vel_n_m_s << std::endl;
			std::cout << "vel_e_m_s: " << msg->vel_e_m_s << std::endl;
			std::cout << "vel_d_m_s: " << msg->vel_d_m_s << std::endl;
			std::cout << "cog_rad: " << msg->cog_rad << std::endl;
			std::cout << "vel_ned_valid: " << msg->vel_ned_valid << std::endl;
			std::cout << "timestamp_time_relative: " << msg->timestamp_time_relative << std::endl;
			std::cout << "time_utc_usec: " << msg->time_utc_usec << std::endl;
			std::cout << "satellites_used: " << msg->satellites_used << std::endl;
			std::cout << "heading: " << msg->heading << std::endl;
			std::cout << "heading_offset: " << msg->heading_offset << std::endl;
		});
	}

private:
	rclcpp::Subscription<px4_msgs::msg::VehicleGpsPosition>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
	std::cout << "Starting vehicle_global_position listener node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<VehicleGpsPositionListener>());

	rclcpp::shutdown();
	return 0;
}
