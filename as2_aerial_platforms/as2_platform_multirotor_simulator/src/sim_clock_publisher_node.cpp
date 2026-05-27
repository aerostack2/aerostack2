// Copyright 2026 Universidad Politécnica de Madrid
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission.
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
 *  \file       sim_clock_publisher_node.cpp
 *  @author     Rafael Perez-Segui <r.psegui@upm.es>
 *  \brief      External `/clock` authority for faster-than-real-time
 *              simulation of the multirotor platform.
 *
 *  Parameters
 *  ----------
 *  - `frequency` (double, Hz, default 1000.0): nominal SIM-time publish
 *    rate. Each tick advances sim_time by `1 / frequency` seconds.
 *  - `rtf` (double, default 1.0): real-time factor. Sim time advances
 *    `rtf` times faster than wall time, so the WALL-clock publish rate
 *    is `frequency * rtf` Hz.
 *
 *  The node forces its own `use_sim_time` to `false` (it is the clock
 *  authority — its wall_timer cannot be bound to its own /clock).
 *
 *  Time advances by a fixed sim step per tick rather than being
 *  derived from `steady_clock::now()` so that downstream consumers see
 *  jitter-free, monotonically-increasing sim-time deltas.
 *********************************************************************************************/

#include <chrono>
#include <cstdint>

#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

namespace as2_platform_multirotor_simulator
{

class SimClockPublisher : public rclcpp::Node
{
public:
  SimClockPublisher()
  : Node("sim_clock_publisher")
  {
    declare_parameter<double>("frequency", 1000.0);
    declare_parameter<double>("rtf", 1.0);

    // We are the clock authority — we MUST tick on wall time, otherwise
    // we'd be waiting on a /clock we haven't published yet.
    set_parameter(rclcpp::Parameter("use_sim_time", false));

    frequency_hz_ = get_parameter("frequency").as_double();
    rtf_ = get_parameter("rtf").as_double();
    if (frequency_hz_ <= 0.0) {
      RCLCPP_ERROR(get_logger(), "frequency must be > 0 (got %.6f), aborting", frequency_hz_);
      throw std::invalid_argument("frequency must be > 0");
    }
    if (rtf_ <= 0.0) {
      RCLCPP_ERROR(get_logger(), "rtf must be > 0 (got %.6f), aborting", rtf_);
      throw std::invalid_argument("rtf must be > 0");
    }

    sim_dt_s_ = 1.0 / frequency_hz_;
    const double wall_dt_s = sim_dt_s_ / rtf_;
    const auto wall_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(wall_dt_s));

    // /clock convention used by gz_ros2_bridge / gazebo_ros: keep_last 1,
    // RELIABLE, VOLATILE. Required for nodes that subscribe with the
    // default sim-time-source profile.
    publisher_ = create_publisher<rosgraph_msgs::msg::Clock>(
      "/clock", rclcpp::QoS(1).reliable());

    timer_ = create_wall_timer(
      wall_period,
      std::bind(&SimClockPublisher::tick, this));

    RCLCPP_INFO(
      get_logger(),
      "sim_clock_publisher: frequency=%.1f Hz sim, RTF=%.2f, wall rate=%.1f Hz",
      frequency_hz_, rtf_, frequency_hz_ * rtf_);
  }

private:
  void tick()
  {
    rosgraph_msgs::msg::Clock msg;
    msg.clock.sec = static_cast<int32_t>(sim_time_s_);
    msg.clock.nanosec =
      static_cast<uint32_t>((sim_time_s_ - msg.clock.sec) * 1.0e9);
    publisher_->publish(msg);
    sim_time_s_ += sim_dt_s_;
  }

  double frequency_hz_{1000.0};
  double rtf_{1.0};
  double sim_dt_s_{0.001};
  double sim_time_s_{0.0};
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace as2_platform_multirotor_simulator

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<as2_platform_multirotor_simulator::SimClockPublisher>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    fprintf(stderr, "sim_clock_publisher_node: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
