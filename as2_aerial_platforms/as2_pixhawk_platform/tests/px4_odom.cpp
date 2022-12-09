#include <iomanip>
#include <iostream>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

void convert(tf2::Quaternion q_ned) {
  tf2::Matrix3x3 ned2enu(0, 1, 0, 1, 0, 0, 0, 0, -1);
  tf2::Quaternion q_ned2enu;
  ned2enu.getRotation(q_ned2enu);

  tf2::Matrix3x3 frd2flu(1, 0, 0, 0, -1, 0, 0, 0, -1);
  tf2::Quaternion q_frd2flu;
  frd2flu.getRotation(q_frd2flu);

  tf2::Quaternion q_enu = q_ned2enu * q_ned * q_frd2flu;

  double roll, pitch, yaw;
  tf2::Matrix3x3(q_enu).getRPY(roll, pitch, yaw);

  std::cout << std::setprecision(2) << std::fixed;
  std::cout << "roll: " << roll * (180 / 3.1416) << ", pitch: " << pitch * (180 / 3.1416)
            << ", yaw: " << yaw * (180 / 3.1416) << std::endl;
  return;
}

class MinimalSubscriber : public rclcpp::Node {
public:
  MinimalSubscriber() : Node("minimal_subscriber") {
    px4_odometry_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
        "fmu/vehicle_odometry/out", rclcpp::SensorDataQoS(),
        std::bind(&MinimalSubscriber::px4odometryCallback, this, std::placeholders::_1));
  }

private:
  void px4odometryCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
    tf2::Quaternion q_msg(msg->q[1], msg->q[2], msg->q[3], msg->q[0]);
    convert(q_msg);
    return;
  }
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_odometry_sub_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}