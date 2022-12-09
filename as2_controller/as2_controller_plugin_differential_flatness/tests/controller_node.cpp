#include <chrono>
#include <functional>
#include <memory>

#include <as2_msgs/msg/thrust.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include "as2_core/names/services.hpp"
#include "as2_core/names/topics.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using namespace std::chrono_literals;

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pid_controller/PID_3D.hpp>
#include <vector>

struct Acro_command {
  Eigen::Vector3d PQR;
  double thrust;
};

// double mass_                         = 0.82;
// Eigen::Vector3d gravitational_accel_ = Eigen::Vector3d(0, 0, 9.81);
// Eigen::Matrix3d Kp_ang_mat_          = Eigen::Matrix3d::Zero();

// Eigen::Vector3d getForce(pid_controller::PIDController3D &pid_handler,
//                          const double &_dt,
//                          const Eigen::Vector3d &_pos_state,
//                          const Eigen::Vector3d &_vel_state,
//                          const Eigen::Vector3d &_pos_reference,
//                          const Eigen::Vector3d &_vel_reference,
//                          const Eigen::Vector3d &_acc_reference) {
//   // Compute the error force contribution
//   // Eigen::Vector3d force_error =
//   //     pid_handler_->computeControl(_dt, _pos_state, _pos_reference, _vel_state,
//   _vel_reference); Eigen::Vector3d force_error =
//       pid_handler.computeControl(_pos_state, _pos_reference, _vel_state, _vel_reference);

//   // Compute acceleration reference contribution
//   Eigen::Vector3d force_acceleration = mass_ * _acc_reference;

//   // Compute gravity compensation
//   Eigen::Vector3d force_gravity = mass_ * gravitational_accel_;

//   // Return desired force with the gravity compensation
//   Eigen::Vector3d desired_force = force_error + force_acceleration + force_gravity;
//   return desired_force;
// }

// Acro_command computeTrajectoryControl(pid_controller::PIDController3D &pid_handler,
//                                       const double &_dt,
//                                       const Eigen::Vector3d &_pos_state,
//                                       const Eigen::Vector3d &_vel_state,
//                                       const tf2::Quaternion &_attitude_state,
//                                       const Eigen::Vector3d &_pos_reference,
//                                       const Eigen::Vector3d &_vel_reference,
//                                       const Eigen::Vector3d &_acc_reference,
//                                       const double &_yaw_angle_reference) {
//   Eigen::Vector3d desired_force = getForce(pid_handler, _dt, _pos_state, _vel_state,
//   _pos_reference,
//                                            _vel_reference, _acc_reference);

//   // Compute the desired attitude
//   tf2::Matrix3x3 rot_matrix_tf2(_attitude_state);

//   Eigen::Matrix3d rot_matrix;
//   rot_matrix << rot_matrix_tf2[0][0], rot_matrix_tf2[0][1], rot_matrix_tf2[0][2],
//       rot_matrix_tf2[1][0], rot_matrix_tf2[1][1], rot_matrix_tf2[1][2], rot_matrix_tf2[2][0],
//       rot_matrix_tf2[2][1], rot_matrix_tf2[2][2];

//   Eigen::Vector3d xc_des(cos(_yaw_angle_reference), sin(_yaw_angle_reference), 0);

//   Eigen::Vector3d zb_des = desired_force.normalized();
//   Eigen::Vector3d yb_des = zb_des.cross(xc_des).normalized();
//   Eigen::Vector3d xb_des = yb_des.cross(zb_des).normalized();

//   // Compute the rotation matrix desidered
//   Eigen::Matrix3d R_des;
//   R_des.col(0) = xb_des;
//   R_des.col(1) = yb_des;
//   R_des.col(2) = zb_des;

//   // Compute the rotation matrix error
//   Eigen::Matrix3d Mat_e_rot = (R_des.transpose() * rot_matrix - rot_matrix.transpose() * R_des);

//   Eigen::Vector3d V_e_rot(Mat_e_rot(2, 1), Mat_e_rot(0, 2), Mat_e_rot(1, 0));
//   Eigen::Vector3d E_rot = (1.0f / 2.0f) * V_e_rot;

//   Acro_command acro_command;
//   acro_command.thrust = (float)desired_force.dot(rot_matrix.col(2).normalized());
//   acro_command.PQR    = -Kp_ang_mat_ * E_rot;

//   return acro_command;
// }

// void test() {
//   pid_controller::PIDController3D pid_handler = pid_controller::PIDController3D();

//   pid_handler.setResetIntegralSaturationFlag(false);
//   pid_handler.setAntiWindup(1.0);
//   pid_handler.setAlpha(0.1);
//   pid_handler.setGainKpX(6.0);
//   pid_handler.setGainKpY(6.0);
//   pid_handler.setGainKpZ(6.0);
//   pid_handler.setGainKiX(0.01);
//   pid_handler.setGainKiY(0.01);
//   pid_handler.setGainKiZ(0.01);
//   pid_handler.setGainKdX(3.0);
//   pid_handler.setGainKdY(3.0);
//   pid_handler.setGainKdZ(3.0);

//   double dt = 0.01;

//   Eigen::Vector3d position_reference(0, 0, 1);
//   Eigen::Vector3d velocity_reference(0, 0, 0);
//   Eigen::Vector3d acceleration_reference(0, 0, 0);
//   double yaw_angle_reference = 0.0;

//   double height = 0.0;
//   for (int i = 0; i < 100; i++) {
//     height += 0.01;

//     Eigen::Vector3d position_state(0, 0, height);
//     Eigen::Vector3d velocity_state(0, 0, 0);
//     tf2::Quaternion attitude_state(0, 0, 0, 1);

//     Acro_command acro_command = computeTrajectoryControl(
//         pid_handler, dt, position_state, velocity_state, attitude_state, position_reference,
//         velocity_reference, acceleration_reference, yaw_angle_reference);

//     // std::cout << "PQR: " << acro_command.PQR << std::endl;
//     std::cout << "Height error: " << position_reference(2) - position_state(2)
//               << ", Thrust: " << acro_command.thrust << std::endl;
//   }

//   return;
// }

class MinimalPublisher : public rclcpp::Node {
public:
  MinimalPublisher() : Node("PX4_test"), count_(0) {
    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        as2_names::topics::actuator_command::twist, as2_names::topics::actuator_command::qos);
    thrust_pub_ = this->create_publisher<as2_msgs::msg::Thrust>(
        as2_names::topics::actuator_command::thrust, as2_names::topics::actuator_command::qos);

    ref_traj_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectoryPoint>(
        as2_names::topics::motion_reference::trajectory, as2_names::topics::motion_reference::qos,
        std::bind(&MinimalPublisher::ref_traj_callback, this, std::placeholders::_1));

    twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        as2_names::topics::self_localization::twist, as2_names::topics::self_localization::qos,
        std::bind(&MinimalPublisher::state_callback, this, std::placeholders::_1));

    pid_handler_ = std::make_shared<pid_controller::PIDController3D>();

    pid_handler_->setResetIntegralSaturationFlag(false);
    pid_handler_->setAntiWindup(1.0);
    pid_handler_->setAlpha(0.1);
    pid_handler_->setGainKpX(6.0);
    pid_handler_->setGainKpY(6.0);
    pid_handler_->setGainKpZ(6.0);
    pid_handler_->setGainKiX(0.01);
    pid_handler_->setGainKiY(0.01);
    pid_handler_->setGainKiZ(0.01);
    pid_handler_->setGainKdX(3.0);
    pid_handler_->setGainKdY(3.0);
    pid_handler_->setGainKdZ(3.0);

    timer_ = this->create_wall_timer(10ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback() {
    double dt = 0.1;

    double yaw_angle_reference = 0.0;
    Eigen::Vector3d acceleration_reference(0, 0, 0);

    control_command_ = computeTrajectoryControl(dt, position_state, velocity_state, attitude_state,
                                                position_reference, velocity_reference,
                                                acceleration_reference, yaw_angle_reference);

    geometry_msgs::msg::TwistStamped twist_msg;
    as2_msgs::msg::Thrust thrust_msg;
    getOutput(twist_msg, thrust_msg);
    twist_pub_->publish(twist_msg);
    thrust_pub_->publish(thrust_msg);
  };

  bool getOutput(geometry_msgs::msg::TwistStamped &twist_msg, as2_msgs::msg::Thrust &thrust_msg) {
    std::string enu_frame_id_ = "odom";
    std::string flu_frame_id_ = "base_link";

    twist_msg.header.stamp    = this->now();
    twist_msg.header.frame_id = flu_frame_id_;
    twist_msg.twist.angular.x = control_command_.PQR.x();
    twist_msg.twist.angular.y = control_command_.PQR.y();
    twist_msg.twist.angular.z = control_command_.PQR.z();

    thrust_msg.header.stamp    = this->now();
    thrust_msg.header.frame_id = flu_frame_id_;
    thrust_msg.thrust          = control_command_.thrust;
    return true;
  };

  Eigen::Vector3d getForce(const double &_dt,
                           const Eigen::Vector3d &_pos_state,
                           const Eigen::Vector3d &_vel_state,
                           const Eigen::Vector3d &_pos_reference,
                           const Eigen::Vector3d &_vel_reference,
                           const Eigen::Vector3d &_acc_reference) {
    // Compute the error force contribution
    Eigen::Vector3d force_error =
        pid_handler_->computeControl(_pos_state, _pos_reference, _vel_state, _vel_reference);

    // Compute acceleration reference contribution
    Eigen::Vector3d force_acceleration = mass_ * _acc_reference;

    // Compute gravity compensation
    Eigen::Vector3d force_gravity = mass_ * gravitational_accel_;

    // Return desired force with the gravity compensation
    Eigen::Vector3d desired_force = force_error + force_acceleration + force_gravity;
    return desired_force;
  }

  Acro_command computeTrajectoryControl(const double &_dt,
                                        const Eigen::Vector3d &_pos_state,
                                        const Eigen::Vector3d &_vel_state,
                                        const tf2::Quaternion &_attitude_state,
                                        const Eigen::Vector3d &_pos_reference,
                                        const Eigen::Vector3d &_vel_reference,
                                        const Eigen::Vector3d &_acc_reference,
                                        const double &_yaw_angle_reference) {
    Eigen::Vector3d desired_force =
        getForce(_dt, _pos_state, _vel_state, _pos_reference, _vel_reference, _acc_reference);

    // Compute the desired attitude
    tf2::Matrix3x3 rot_matrix_tf2(_attitude_state);

    Eigen::Matrix3d rot_matrix;
    rot_matrix << rot_matrix_tf2[0][0], rot_matrix_tf2[0][1], rot_matrix_tf2[0][2],
        rot_matrix_tf2[1][0], rot_matrix_tf2[1][1], rot_matrix_tf2[1][2], rot_matrix_tf2[2][0],
        rot_matrix_tf2[2][1], rot_matrix_tf2[2][2];

    Eigen::Vector3d xc_des(cos(_yaw_angle_reference), sin(_yaw_angle_reference), 0);

    Eigen::Vector3d zb_des = desired_force.normalized();
    Eigen::Vector3d yb_des = zb_des.cross(xc_des).normalized();
    Eigen::Vector3d xb_des = yb_des.cross(zb_des).normalized();

    // Compute the rotation matrix desidered
    Eigen::Matrix3d R_des;
    R_des.col(0) = xb_des;
    R_des.col(1) = yb_des;
    R_des.col(2) = zb_des;

    // Compute the rotation matrix error
    Eigen::Matrix3d Mat_e_rot = (R_des.transpose() * rot_matrix - rot_matrix.transpose() * R_des);

    Eigen::Vector3d V_e_rot(Mat_e_rot(2, 1), Mat_e_rot(0, 2), Mat_e_rot(1, 0));
    Eigen::Vector3d E_rot = (1.0f / 2.0f) * V_e_rot;

    Acro_command acro_command;
    acro_command.thrust = (float)desired_force.dot(rot_matrix.col(2).normalized());
    acro_command.PQR    = -Kp_ang_mat_ * E_rot;

    return acro_command;
  }

  double mass_                         = 0.82;
  Eigen::Vector3d gravitational_accel_ = Eigen::Vector3d(0, 0, 9.81);
  Eigen::Matrix3d Kp_ang_mat_          = Eigen::Matrix3d::Zero();

  Acro_command control_command_;

  std::shared_ptr<pid_controller::PIDController3D> pid_handler_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<as2_msgs::msg::Thrust>::SharedPtr thrust_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr ref_traj_sub_;
  size_t count_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}