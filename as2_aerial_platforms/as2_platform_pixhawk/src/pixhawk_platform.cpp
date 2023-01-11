/*!*******************************************************************************************
 *  \file       pixhawk_platform.cpp
 *  \brief      Implementation of PX4 Autopilot UAV platform
 *  \authors    Miguel Fernández Cortizas
 *              David Pérez Saura
 *              Rafael Pérez Seguí
 *              Pedro Arias Pérez
 *
 *  \copyright  Copyright (c) 2022 Universidad Politécnica de Madrid
 *              All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/

#include "pixhawk_platform.hpp"

PixhawkPlatform::PixhawkPlatform() : as2::AerialPlatform() {
  configureSensors();

  base_link_frame_id_ = as2::tf::generateTfName(this, "base_link");
  odom_frame_id_      = as2::tf::generateTfName(this, "odom");

  this->declare_parameter<float>("mass");
  mass_ = this->get_parameter("mass").as_double();

  this->declare_parameter<float>("max_thrust");
  max_thrust_ = this->get_parameter("max_thrust").as_double();

  this->declare_parameter<float>("min_thrust");
  min_thrust_ = this->get_parameter("min_thrust").as_double();

  this->declare_parameter<bool>("external_odom");
  external_odom_ = this->get_parameter("external_odom").as_bool();

  RCLCPP_INFO(this->get_logger(), "Mass: %f", mass_);
  RCLCPP_INFO(this->get_logger(), "Max thrust: %f", max_thrust_);
  RCLCPP_INFO(this->get_logger(), "Min thrust: %f", min_thrust_);
  RCLCPP_INFO(this->get_logger(), "Simulation mode: %s",
              this->get_parameter("use_sim_time").as_bool() ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "External odometry mode: %s", external_odom_ ? "true" : "false");

  as2::frame::eulerToQuaternion(M_PI, 0.0, M_PI_2, q_ned_to_enu_);
  q_enu_to_ned_ = q_ned_to_enu_;
  as2::frame::eulerToQuaternion(M_PI, 0.0, 0.0, q_aircraft_to_baselink_);
  q_baselink_to_aircraft_ = q_aircraft_to_baselink_;

  // declare PX4 subscribers
  px4_imu_sub_ = this->create_subscription<px4_msgs::msg::SensorCombined>(
      "fmu/sensor_combined/out", rclcpp::SensorDataQoS(),
      std::bind(&PixhawkPlatform::px4imuCallback, this, std::placeholders::_1));

  px4_timesync_sub_ = this->create_subscription<px4_msgs::msg::Timesync>(
      "fmu/timesync/out", rclcpp::SensorDataQoS(),
      [this](const px4_msgs::msg::Timesync::UniquePtr msg) { timestamp_.store(msg->timestamp); });

  px4_vehicle_control_mode_sub_ = this->create_subscription<px4_msgs::msg::VehicleControlMode>(
      "fmu/vehicle_control_mode/out", rclcpp::SensorDataQoS(),
      std::bind(&PixhawkPlatform::px4VehicleControlModeCallback, this, std::placeholders::_1));

  px4_gps_sub_ = this->create_subscription<px4_msgs::msg::SensorGps>(
      "fmu/sensor_gps/out", rclcpp::SensorDataQoS(),
      std::bind(&PixhawkPlatform::px4GpsCallback, this, std::placeholders::_1));

  px4_battery_sub_ = this->create_subscription<px4_msgs::msg::BatteryStatus>(
      "fmu/battery_status/out", rclcpp::SensorDataQoS(),
      std::bind(&PixhawkPlatform::px4BatteryCallback, this, std::placeholders::_1));

  px4_odometry_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
      "fmu/vehicle_odometry/out", rclcpp::SensorDataQoS(),
      std::bind(&PixhawkPlatform::px4odometryCallback, this, std::placeholders::_1));
  tf_handler_ = std::make_shared<as2::tf::TfHandler>(this);

  if (external_odom_) {
    // In real flights, the odometry is published by the onboard computer.
    external_odometry_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        this->generate_global_name(as2_names::topics::self_localization::twist),
        as2_names::topics::self_localization::qos,
        std::bind(&PixhawkPlatform::externalOdomCb, this, std::placeholders::_1));

    static auto px4_publish_vo_timer = this->create_wall_timer(
        std::chrono::milliseconds(10), [this]() { this->PX4publishVisualOdometry(); });
  }

  // declare PX4 publishers
  px4_offboard_control_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
      "fmu/offboard_control_mode/in", rclcpp::SensorDataQoS());
  px4_trajectory_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      "fmu/trajectory_setpoint/in", rclcpp::SensorDataQoS());
  px4_vehicle_attitude_setpoint_pub_ =
      this->create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>(
          "fmu/vehicle_attitude_setpoint/in", rclcpp::SensorDataQoS());
  px4_vehicle_rates_setpoint_pub_ = this->create_publisher<px4_msgs::msg::VehicleRatesSetpoint>(
      "fmu/vehicle_rates_setpoint/in", rclcpp::SensorDataQoS());
  px4_vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
      "fmu/vehicle_command/in", rclcpp::SensorDataQoS());
  px4_visual_odometry_pub_ = this->create_publisher<px4_msgs::msg::VehicleVisualOdometry>(
      "fmu/vehicle_visual_odometry/in", rclcpp::SensorDataQoS());

  px4_manual_control_switches_pub_ = this->create_publisher<px4_msgs::msg::ManualControlSwitches>(
      "fmu/manual_control_switches/in", rclcpp::SensorDataQoS());
}

void PixhawkPlatform::configureSensors() {
  imu_sensor_ptr_     = std::make_unique<as2::sensors::Imu>("imu", this);
  battery_sensor_ptr_ = std::make_unique<as2::sensors::Battery>("battery", this);
  gps_sensor_ptr_     = std::make_unique<as2::sensors::GPS>("gps", this);

  odometry_raw_estimation_ptr_ =
      std::make_unique<as2::sensors::Sensor<nav_msgs::msg::Odometry>>("odom", this);
}

bool PixhawkPlatform::ownSetArmingState(bool state) {
  if (state) {
    this->PX4arm();
  } else {
    set_disarm_ = true;
    // TODO: wait for takeoff_status to be able to PX4 disarm
    this->PX4disarm();
  }
  return true;
}

bool PixhawkPlatform::ownSetOffboardControl(bool offboard) {
  // TODO: CREATE A DEFAULT CONTROL MODE FOR BEING ABLE TO SWITCH TO OFFBOARD
  // MODE BEFORE RUNNING THE CONTROLLER

  if (offboard == false) {
    RCLCPP_ERROR(this->get_logger(),
                 "Turning into MANUAL Mode is not allowed from the onboard computer");
    return false;
  }

  px4_offboard_control_mode_ = px4_msgs::msg::OffboardControlMode();  // RESET CONTROL MODE
  px4_offboard_control_mode_.body_rate = true;
  resetRatesSetpoint();

  RCLCPP_DEBUG(this->get_logger(), "Switching to OFFBOARD mode");
  // Following PX4 offboard guidelines
  rclcpp::Rate r(100);
  for (int i = 0; i < 100; i++) {
    PX4publishRatesSetpoint();
    r.sleep();
  }
  PX4publishOffboardControlMode();
  return true;
}

bool PixhawkPlatform::ownSetPlatformControlMode(const as2_msgs::msg::ControlMode &msg) {
  px4_offboard_control_mode_ = px4_msgs::msg::OffboardControlMode();  // RESET CONTROL MODE

  /* PIXHAWK CONTROL MODES:
  px4_offboard_control_mode_.position      ->  x,y,z
  px4_offboard_control_mode_.velocity      ->  vx,vy,vz
  px4_offboard_control_mode_.acceleration  ->  ax,ay,az
  px4_offboard_control_mode_.attitude      ->  q(r,p,y) + T(tx,ty,tz) in
  multicopters tz = -Collective_Thrust px4_offboard_control_mode_.body_rate ->
  p ,q ,r  + T(tx,ty,tz) in multicopters tz = -Collective_Thrust */

  switch (msg.control_mode) {
    case as2_msgs::msg::ControlMode::POSITION: {
      px4_offboard_control_mode_.position = true;
      RCLCPP_INFO(this->get_logger(), "POSITION_MODE ENABLED");
    } break;
    case as2_msgs::msg::ControlMode::SPEED: {
      px4_offboard_control_mode_.velocity = true;
      RCLCPP_INFO(this->get_logger(), "SPEED_MODE ENABLED");
    } break;
    case as2_msgs::msg::ControlMode::ATTITUDE: {
      px4_offboard_control_mode_.attitude = true;
      RCLCPP_INFO(this->get_logger(), "ATTITUDE_MODE ENABLED");
    } break;
    // TODO ACCEL MODE NOT IMPLEMENTED
    // case as2_msgs::msg::ControlMode::ACCEL_MODE: {
    //   px4_offboard_control_mode_.acceleration = true;
    //   RCLCPP_INFO(this->get_logger(), "ACCEL_MODE ENABLED");
    // } break;
    case as2_msgs::msg::ControlMode::ACRO: {
      px4_offboard_control_mode_.body_rate = true;
      RCLCPP_INFO(this->get_logger(), "ACRO_MODE ENABLED");
    } break;
    default:
      RCLCPP_WARN(this->get_logger(), "CONTROL MODE %d NOT SUPPORTED", msg.control_mode);
      has_mode_settled_ = false;
      return false;
  }

  has_mode_settled_ = true;
  return true;
}

float yawEnuToAircraft(geometry_msgs::msg::PoseStamped command_pose_msg) {
  tf2::Quaternion q_input(command_pose_msg.pose.orientation.x, command_pose_msg.pose.orientation.y,
                          command_pose_msg.pose.orientation.z, command_pose_msg.pose.orientation.w);

  tf2::Matrix3x3 m_input(q_input);
  double roll, pitch, yaw;
  m_input.getRPY(roll, pitch, yaw);

  return -yaw + M_PI_2;
}

void PixhawkPlatform::sendCommand() {
  // Actuator commands are published continously
  if (!getArmingState()) {
    return;
  }

  if (this->getFlagSimulationMode()) {
    if (!getOffboardMode()) return;
  } else {
    if ((!getOffboardMode() || !has_mode_settled_) && !manual_from_operator_) {
      px4_offboard_control_mode_ = px4_msgs::msg::OffboardControlMode();  // RESET CONTROL MODE
      px4_offboard_control_mode_.body_rate = true;

      resetRatesSetpoint();
      px4_rates_setpoint_.thrust_body[2] = -0.1f;

      PX4publishRatesSetpoint();
      return;
    }
  }
  ownSendCommand();
  return;
}

bool PixhawkPlatform::ownSendCommand() {
  as2_msgs::msg::ControlMode platform_control_mode = this->getControlMode();

  // Switch case to set setpoint
  switch (platform_control_mode.control_mode) {
    case as2_msgs::msg::ControlMode::POSITION: {
      this->resetTrajectorySetpoint();
      if (platform_control_mode.yaw_mode == as2_msgs::msg::ControlMode::YAW_ANGLE) {
        px4_trajectory_setpoint_.yaw = yawEnuToAircraft(this->command_pose_msg_);
      } else {
        // ENU --> NED
        px4_trajectory_setpoint_.yawspeed = -command_twist_msg_.twist.angular.z;
      }

      Eigen::Vector3d position_enu;
      position_enu.x() = this->command_pose_msg_.pose.position.x;
      position_enu.y() = this->command_pose_msg_.pose.position.y;
      position_enu.z() = this->command_pose_msg_.pose.position.z;

      // TODO: px4_ros_com done
      const Eigen::PermutationMatrix<3> ned_enu_reflection_xy(Eigen::Vector3i(1, 0, 2));
      const Eigen::DiagonalMatrix<double, 3> ned_enu_reflection_z(1, 1, -1);
      Eigen::Vector3d position_ned = ned_enu_reflection_xy * (ned_enu_reflection_z * position_enu);
      // Eigen::Vector3d position_ned = px4_ros_com::frame_transforms::transform_static_frame(
      //     position_enu, px4_ros_com::frame_transforms::StaticTF::ENU_TO_NED);

      px4_trajectory_setpoint_.x = position_ned.x();
      px4_trajectory_setpoint_.y = position_ned.y();
      px4_trajectory_setpoint_.z = position_ned.z();
    } break;
    case as2_msgs::msg::ControlMode::SPEED: {
      this->resetTrajectorySetpoint();
      if (platform_control_mode.yaw_mode == as2_msgs::msg::ControlMode::YAW_ANGLE) {
        px4_trajectory_setpoint_.yaw = yawEnuToAircraft(this->command_pose_msg_);
      } else {
        // ENU --> NED
        px4_trajectory_setpoint_.yawspeed = -command_twist_msg_.twist.angular.z;
      }

      Eigen::Vector3d speed_enu;
      speed_enu.x() = this->command_twist_msg_.twist.linear.x;
      speed_enu.y() = this->command_twist_msg_.twist.linear.y;
      speed_enu.z() = this->command_twist_msg_.twist.linear.z;

      // TODO: px4_ros_com done
      const Eigen::PermutationMatrix<3> ned_enu_reflection_xy(Eigen::Vector3i(1, 0, 2));
      const Eigen::DiagonalMatrix<double, 3> ned_enu_reflection_z(1, 1, -1);
      Eigen::Vector3d speed_ned = ned_enu_reflection_xy * (ned_enu_reflection_z * speed_enu);
      // Eigen::Vector3d speed_ned = px4_ros_com::frame_transforms::transform_static_frame(
      //     speed_enu, px4_ros_com::frame_transforms::StaticTF::ENU_TO_NED);

      px4_trajectory_setpoint_.vx = speed_ned.x();
      px4_trajectory_setpoint_.vy = speed_ned.y();
      px4_trajectory_setpoint_.vz = speed_ned.z();
    } break;
    case as2_msgs::msg::ControlMode::ATTITUDE: {
      this->resetAttitudeSetpoint();
      if (platform_control_mode.yaw_mode == as2_msgs::msg::ControlMode::YAW_SPEED) {
        RCLCPP_WARN_ONCE(this->get_logger(), "Yaw Speed control not supported on ATTITUDE mode");
      }

      Eigen::Quaterniond q_enu;
      q_enu.w() = this->command_pose_msg_.pose.orientation.w;
      q_enu.x() = this->command_pose_msg_.pose.orientation.x;
      q_enu.y() = this->command_pose_msg_.pose.orientation.y;
      q_enu.z() = this->command_pose_msg_.pose.orientation.z;

      // TODO: px4_ros_com done
      Eigen::Quaterniond q_ned      = q_enu_to_ned_ * q_enu;
      Eigen::Quaterniond q_aircraft = q_ned * q_baselink_to_aircraft_;
      // Eigen::Quaterniond q_ned = px4_ros_com::frame_transforms::transform_orientation(
      //     q_enu, px4_ros_com::frame_transforms::StaticTF::ENU_TO_NED);
      // Eigen::Quaterniond q_aircraft = px4_ros_com::frame_transforms::transform_orientation(
      //     q_ned, px4_ros_com::frame_transforms::StaticTF::BASELINK_TO_AIRCRAFT);

      px4_attitude_setpoint_.q_d[0] = q_aircraft.w();
      px4_attitude_setpoint_.q_d[1] = q_aircraft.x();
      px4_attitude_setpoint_.q_d[2] = q_aircraft.y();
      px4_attitude_setpoint_.q_d[3] = q_aircraft.z();

      // minus because px4 uses NED (Z is downwards)
      if (command_thrust_msg_.thrust < min_thrust_) {
        px4_attitude_setpoint_.thrust_body[2] = -min_thrust_;
        if (this->set_disarm_) {
          px4_rates_setpoint_.thrust_body[2] = 0.0f;
        }
      } else {
        px4_attitude_setpoint_.thrust_body[2] = -command_thrust_msg_.thrust / max_thrust_;
      }
    } break;
    case as2_msgs::msg::ControlMode::ACRO: {
      this->resetRatesSetpoint();
      if (platform_control_mode.yaw_mode == as2_msgs::msg::ControlMode::YAW_ANGLE) {
        RCLCPP_WARN_ONCE(this->get_logger(), "Yaw Angle control not supported on ACRO mode");
      }

      // FLU --> FRD
      px4_rates_setpoint_.roll  = command_twist_msg_.twist.angular.x;
      px4_rates_setpoint_.pitch = -command_twist_msg_.twist.angular.y;
      px4_rates_setpoint_.yaw   = -command_twist_msg_.twist.angular.z;

      // minus because px4 uses NED (Z is downwards)
      if (command_thrust_msg_.thrust < min_thrust_) {
        px4_rates_setpoint_.thrust_body[2] = -min_thrust_;
        if (this->set_disarm_) {
          px4_rates_setpoint_.thrust_body[2] = 0.0f;
        }
      } else {
        px4_rates_setpoint_.thrust_body[2] = -command_thrust_msg_.thrust / max_thrust_;
      }
    } break;
    // case as2_msgs::msg::ControlMode::ACCEL :{
    //   // Mode to implement
    //   RCLCPP_ERROR(this->get_logger(), "ACCELERATION CONTROL MODE is not
    //   supported yet "); return false;
    //   }
    //   break;
    default:
      return false;
  }

  if (px4_offboard_control_mode_.attitude) {
    this->PX4publishAttitudeSetpoint();
  } else if (px4_offboard_control_mode_.body_rate) {
    this->PX4publishRatesSetpoint();
  } else if (px4_offboard_control_mode_.position || px4_offboard_control_mode_.velocity ||
             px4_offboard_control_mode_.acceleration) {
    this->PX4publishTrajectorySetpoint();
  }
  return true;
}

void PixhawkPlatform::ownKillSwitch() {
  RCLCPP_ERROR(this->get_logger(), "KILL SWITCH TRIGGERED");
  px4_msgs::msg::ManualControlSwitches kill_switch_msg;
  kill_switch_msg.kill_switch = true;
  px4_manual_control_switches_pub_->publish(kill_switch_msg);
}

void PixhawkPlatform::ownStopPlatform() { RCLCPP_WARN(this->get_logger(), "NOT IMPLEMENTED"); }

void PixhawkPlatform::resetTrajectorySetpoint() {
  px4_trajectory_setpoint_.x = NAN;
  px4_trajectory_setpoint_.y = NAN;
  px4_trajectory_setpoint_.z = NAN;

  px4_trajectory_setpoint_.vx = NAN;
  px4_trajectory_setpoint_.vy = NAN;
  px4_trajectory_setpoint_.vz = NAN;

  px4_trajectory_setpoint_.yaw      = NAN;
  px4_trajectory_setpoint_.yawspeed = NAN;

  px4_trajectory_setpoint_.acceleration = std::array<float, 3>{NAN, NAN, NAN};
  px4_trajectory_setpoint_.jerk         = std::array<float, 3>{NAN, NAN, NAN};
  px4_trajectory_setpoint_.thrust       = std::array<float, 3>{NAN, NAN, NAN};
}

void PixhawkPlatform::resetAttitudeSetpoint() {
  px4_attitude_setpoint_.pitch_body = NAN;
  px4_attitude_setpoint_.roll_body  = NAN;
  px4_attitude_setpoint_.yaw_body   = NAN;

  px4_attitude_setpoint_.q_d         = std::array<float, 4>{0, 0, 0, 1};
  px4_attitude_setpoint_.thrust_body = std::array<float, 3>{0, 0, -min_thrust_};
}

void PixhawkPlatform::resetRatesSetpoint() {
  px4_rates_setpoint_.roll           = 0.0f;
  px4_rates_setpoint_.pitch          = 0.0f;
  px4_rates_setpoint_.yaw            = 0.0f;
  px4_attitude_setpoint_.thrust_body = std::array<float, 3>{0, 0, -min_thrust_};
}

void PixhawkPlatform::externalOdomCb(const geometry_msgs::msg::TwistStamped::SharedPtr _twist_msg) {
  try {
    auto [pose_msg, twist_msg] = tf_handler_->getState(*_twist_msg, base_link_frame_id_,
                                                       odom_frame_id_, base_link_frame_id_);

    odometry_msg_.header.stamp    = twist_msg.header.stamp;
    odometry_msg_.header.frame_id = pose_msg.header.frame_id;   // LOCAL_FRAME_FLU
    odometry_msg_.child_frame_id  = twist_msg.header.frame_id;  // BODY_FRAME_FLU
    odometry_msg_.pose.pose       = pose_msg.pose;
    odometry_msg_.twist.twist     = twist_msg.twist;
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
  }
  return;
}

/** -----------------------------------------------------------------*/
/** ------------------------- PX4 FUNCTIONS -------------------------*/
/** -----------------------------------------------------------------*/

/**
 * @brief Send a command to Arm the vehicle
 */
void PixhawkPlatform::PX4arm() const {
  PX4publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
  RCLCPP_DEBUG(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void PixhawkPlatform::PX4disarm() const {
  PX4publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
  RCLCPP_DEBUG(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *
 */
void PixhawkPlatform::PX4publishOffboardControlMode() {
  PX4publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
  RCLCPP_DEBUG(this->get_logger(), "OFFBOARD mode enabled");
}

/**
 * @brief Publish a trajectory setpoint
 */
void PixhawkPlatform::PX4publishTrajectorySetpoint() {
  px4_trajectory_setpoint_.timestamp   = timestamp_.load();
  px4_offboard_control_mode_.timestamp = timestamp_.load();

  px4_trajectory_setpoint_pub_->publish(px4_trajectory_setpoint_);
  px4_offboard_control_mode_pub_->publish(px4_offboard_control_mode_);
}

/**
 * @brief Publish a attitude setpoint
 */
void PixhawkPlatform::PX4publishAttitudeSetpoint() {
  px4_attitude_setpoint_.timestamp     = timestamp_.load();
  px4_offboard_control_mode_.timestamp = timestamp_.load();

  px4_vehicle_attitude_setpoint_pub_->publish(px4_attitude_setpoint_);
  px4_offboard_control_mode_pub_->publish(px4_offboard_control_mode_);
}

/**
 * @brief Publish a vehicle rates setpoint
 */
void PixhawkPlatform::PX4publishRatesSetpoint() {
  px4_rates_setpoint_.timestamp        = timestamp_.load();
  px4_offboard_control_mode_.timestamp = timestamp_.load();

  px4_vehicle_rates_setpoint_pub_->publish(px4_rates_setpoint_);
  px4_offboard_control_mode_pub_->publish(px4_offboard_control_mode_);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD
 * codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void PixhawkPlatform::PX4publishVehicleCommand(uint16_t command, float param1, float param2) const {
  px4_msgs::msg::VehicleCommand msg{};
  msg.timestamp        = timestamp_.load();
  msg.param1           = param1;
  msg.param2           = param2;
  msg.command          = command;
  msg.target_system    = 1;
  msg.target_component = 1;
  msg.source_system    = 1;
  msg.source_component = 1;
  msg.from_external    = true;

  px4_vehicle_command_pub_->publish(msg);
}

/// @brief See documentation: https://docs.px4.io/v1.13/en/msg_docs/vehicle_odometry.html
void PixhawkPlatform::PX4publishVisualOdometry() {
  // Position in meters. Frame of reference defined by local_frame
  px4_visual_odometry_msg_.local_frame = px4_msgs::msg::VehicleOdometry::LOCAL_FRAME_FRD;
  // FLU --> FRD
  px4_visual_odometry_msg_.x = odometry_msg_.pose.pose.position.x;
  px4_visual_odometry_msg_.y = -odometry_msg_.pose.pose.position.y;
  px4_visual_odometry_msg_.z = -odometry_msg_.pose.pose.position.x;

  // Quaternion rotation from FRD body frame to refernce frame
  Eigen::Quaterniond q_baselink(
      odometry_msg_.pose.pose.orientation.w, odometry_msg_.pose.pose.orientation.x,
      odometry_msg_.pose.pose.orientation.y, odometry_msg_.pose.pose.orientation.z);
  // BASELINK --> AIRCRAFT (FLU --> FRD)

  // TODO: px4_ros_com done
  Eigen::Quaterniond q_aircraft = q_baselink * q_baselink_to_aircraft_;
  // Eigen::Quaterniond q_aircraft =
  // px4_ros_com::frame_transforms::transform_orientation(q_baselink,
  // px4_ros_com::frame_transforms::StaticTF::BASELINK_TO_AIRCRAFT);

  px4_visual_odometry_msg_.q[0] = q_aircraft.w();
  px4_visual_odometry_msg_.q[1] = q_aircraft.x();
  px4_visual_odometry_msg_.q[2] = q_aircraft.y();
  px4_visual_odometry_msg_.q[3] = q_aircraft.z();

  // Velocity in meters/sec. Frame of reference defined by velocity_frame variable.
  px4_visual_odometry_msg_.velocity_frame = px4_msgs::msg::VehicleOdometry::BODY_FRAME_FRD;
  // FLU --> FRD
  px4_visual_odometry_msg_.vx = odometry_msg_.twist.twist.linear.x;
  px4_visual_odometry_msg_.vy = -odometry_msg_.twist.twist.linear.y;
  px4_visual_odometry_msg_.vz = -odometry_msg_.twist.twist.linear.z;

  // Angular rate in body-fixed frame (rad/s).
  // FLU --> FRD
  px4_visual_odometry_msg_.rollspeed  = odometry_msg_.twist.twist.angular.x;
  px4_visual_odometry_msg_.pitchspeed = -odometry_msg_.twist.twist.angular.y;
  px4_visual_odometry_msg_.yawspeed   = -odometry_msg_.twist.twist.angular.z;

  px4_visual_odometry_msg_.timestamp = timestamp_.load();
  px4_visual_odometry_pub_->publish(px4_visual_odometry_msg_);
}

/** -----------------------------------------------------------------*/
/** ---------------------- SUBSCRIBER CALLBACKS ---------------------*/
/** -----------------------------------------------------------------*/

void PixhawkPlatform::px4imuCallback(const px4_msgs::msg::SensorCombined::SharedPtr msg) {
  auto timestamp = this->get_clock()->now();
  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header.stamp          = timestamp;
  imu_msg.header.frame_id       = base_link_frame_id_;
  imu_msg.linear_acceleration.x = msg->accelerometer_m_s2[0];
  imu_msg.linear_acceleration.y = msg->accelerometer_m_s2[1];
  imu_msg.linear_acceleration.z = msg->accelerometer_m_s2[2];
  imu_msg.angular_velocity.x    = msg->gyro_rad[0];
  imu_msg.angular_velocity.y    = msg->gyro_rad[1];
  imu_msg.angular_velocity.z    = msg->gyro_rad[2];

  imu_sensor_ptr_->updateData(imu_msg);
}

/// @brief See documentation: https://docs.px4.io/v1.13/en/msg_docs/vehicle_odometry.html
/// @param msg vehicle odometry
void PixhawkPlatform::px4odometryCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp    = this->get_clock()->now();
  odom_msg.header.frame_id = odom_frame_id_;
  odom_msg.child_frame_id  = base_link_frame_id_;

  switch (msg->local_frame) {
    case px4_msgs::msg::VehicleOdometry::LOCAL_FRAME_NED:
    case px4_msgs::msg::VehicleOdometry::LOCAL_FRAME_FRD: {
      // as2 understand initial Forward as North, so LOCAL_FRD equals LOCAL_NED

      // Position in meters. Frame of reference defined by local_frame
      Eigen::Vector3d pos_ned(msg->x, msg->y, msg->z);

      // TODO: px4_ros_com done
      const Eigen::PermutationMatrix<3> ned_enu_reflection_xy(Eigen::Vector3i(1, 0, 2));
      const Eigen::DiagonalMatrix<double, 3> ned_enu_reflection_z(1, 1, -1);
      Eigen::Vector3d pos_enu = ned_enu_reflection_xy * (ned_enu_reflection_z * pos_ned);
      // Eigen::Vector3d pos_enu = px4_ros_com::frame_transforms::ned_to_enu_local_frame(pos_ned);

      odom_msg.pose.pose.position.x = pos_enu[0];
      odom_msg.pose.pose.position.y = pos_enu[1];
      odom_msg.pose.pose.position.z = pos_enu[2];

      // Quaternion rotation from FRD body frame to refernce frame (LOCAL_FRAME_NED)
      // q_offset Quaternion rotation from odometry reference frame to navigation frame
      Eigen::Quaterniond q_local_ned(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);

      // TODO: px4_ros_com
      Eigen::Quaterniond q_local_enu = q_ned_to_enu_ * q_local_ned;
      Eigen::Quaterniond q_flu       = q_local_enu * q_aircraft_to_baselink_;

      // Eigen::Quaterniond q_local_enu =
      //     px4_ros_com::frame_transforms::ned_to_enu_orientation(q_local_ned);
      // // AIRCRAFT (FRD): BODY FRD --> BODY FLU
      // Eigen::Quaterniond q_flu =
      //     px4_ros_com::frame_transforms::aircraft_to_baselink_orientation(q_local_enu);

      odom_msg.pose.pose.orientation.w = q_flu.w();
      odom_msg.pose.pose.orientation.x = q_flu.x();
      odom_msg.pose.pose.orientation.y = q_flu.y();
      odom_msg.pose.pose.orientation.z = q_flu.z();
      break;
    }
    default:
      RCLCPP_ERROR(this->get_logger(), "Vehicle Odometry local frame not supported.");
      break;
  }

  // Velocity in meters/sec. Frame of reference defined by velocity_frame variable
  switch (msg->velocity_frame) {
    case px4_msgs::msg::VehicleOdometry::LOCAL_FRAME_NED:
    case px4_msgs::msg::VehicleOdometry::LOCAL_FRAME_FRD: {
      // Convert from NED to FLU
      Eigen::Vector3d vel_ned = Eigen::Vector3d(msg->vx, msg->vy, msg->vz);

      // TODO: px4_ros_com done
      const Eigen::PermutationMatrix<3> ned_enu_reflection_xy(Eigen::Vector3i(1, 0, 2));
      const Eigen::DiagonalMatrix<double, 3> ned_enu_reflection_z(1, 1, -1);
      Eigen::Vector3d vel_enu = ned_enu_reflection_xy * (ned_enu_reflection_z * vel_ned);
      // Eigen::Vector3d vel_enu = px4_ros_com::frame_transforms::ned_to_enu_local_frame(vel_ned);

      Eigen::Quaterniond q_baselink(
          odom_msg.pose.pose.orientation.w, odom_msg.pose.pose.orientation.x,
          odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z);

      // TODO: px4_ros_com done
      Eigen::Affine3d transformation(q_baselink.inverse());
      Eigen::Vector3d vel_flu = transformation * vel_enu;
      // Eigen::Vector3d vel_flu =
      //     px4_ros_com::frame_transforms::transform_frame(vel_enu, q_baselink.inverse());

      odom_msg.twist.twist.linear.x = vel_flu[0];
      odom_msg.twist.twist.linear.y = vel_flu[1];
      odom_msg.twist.twist.linear.z = vel_flu[2];
      break;
    }
    case px4_msgs::msg::VehicleOdometry::BODY_FRAME_FRD: {
      // FRD --> FLU
      odom_msg.twist.twist.linear.x = msg->vx;
      odom_msg.twist.twist.linear.y = -msg->vy;
      odom_msg.twist.twist.linear.z = -msg->vz;
      break;
    }
    default:
      RCLCPP_ERROR(this->get_logger(), "Vehicle Odometry velocity frame not supported.");
      break;
  }

  // Angular rate in body-fixed frame (rad/s): FRD --> FLU
  odom_msg.twist.twist.angular.x = msg->rollspeed;
  odom_msg.twist.twist.angular.y = -msg->pitchspeed;
  odom_msg.twist.twist.angular.z = -msg->yawspeed;

  odometry_raw_estimation_ptr_->updateData(odom_msg);
}

void PixhawkPlatform::px4VehicleControlModeCallback(
    const px4_msgs::msg::VehicleControlMode::SharedPtr msg) {
  static bool last_arm_state      = msg->flag_armed;
  static bool last_offboard_state = msg->flag_control_offboard_enabled;

  this->platform_info_msg_.armed    = msg->flag_armed;
  this->platform_info_msg_.offboard = msg->flag_control_offboard_enabled;

  if (this->platform_info_msg_.offboard != last_offboard_state) {
    if (this->platform_info_msg_.offboard)
      RCLCPP_INFO(this->get_logger(), "OFFBOARD_ENABLED");
    else {
      RCLCPP_INFO(this->get_logger(), "OFFBOARD_DISABLED");
      manual_from_operator_ = true;
    }
    last_offboard_state = this->platform_info_msg_.offboard;
  }

  if (this->platform_info_msg_.armed != last_arm_state) {
    if (this->platform_info_msg_.armed) {
      RCLCPP_INFO(this->get_logger(), "ARMING");
      this->handleStateMachineEvent(as2_msgs::msg::PlatformStateMachineEvent::ARM);
    } else {
      RCLCPP_INFO(this->get_logger(), "DISARMING");
      this->handleStateMachineEvent(as2_msgs::msg::PlatformStateMachineEvent::DISARM);
    }
    last_arm_state = this->platform_info_msg_.armed;
  }
}

void PixhawkPlatform::px4GpsCallback(const px4_msgs::msg::SensorGps::SharedPtr msg) {
  // Reference:
  // https://github.com/PX4/PX4-Autopilot/blob/052adfbfd977abfad5b6f58d5404bba7dd209736/src/modules/mavlink/streams/GPS_RAW_INT.hpp#L58
  // https://github.com/mavlink/mavros/blob/b392c23add8781a67ac90915278fe41086fecaeb/mavros/src/plugins/global_position.cpp#L161

  auto timestamp = this->get_clock()->now();

  sensor_msgs::msg::NavSatFix nav_sat_fix_msg;
  nav_sat_fix_msg.header.stamp = timestamp;

  nav_sat_fix_msg.header.frame_id = "wgs84";
  if (msg->fix_type > 2) {  // At least 3D position
    nav_sat_fix_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  } else {
    nav_sat_fix_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
  }
  nav_sat_fix_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;  // DEFAULT
  nav_sat_fix_msg.latitude       = msg->lat;
  nav_sat_fix_msg.longitude      = msg->lon;
  nav_sat_fix_msg.altitude       = msg->alt_ellipsoid;

  if (!std::isnan(msg->eph) && !std::isnan(msg->epv)) {
    // Position uncertainty --> Diagonal known
    nav_sat_fix_msg.position_covariance.fill(0.0);
    nav_sat_fix_msg.position_covariance[0] = std::pow(msg->eph, 2);
    nav_sat_fix_msg.position_covariance[4] = std::pow(msg->eph, 2);
    nav_sat_fix_msg.position_covariance[8] = std::pow(msg->epv, 2);
    nav_sat_fix_msg.position_covariance_type =
        sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
  } else {
    // UNKOWN
    nav_sat_fix_msg.position_covariance      = {-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    nav_sat_fix_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
  }
  // nav_sat_fix_msg.altitude = nav_sat_fix_msg.altitude / 1e7; TODO: Check
  // altitude
  nav_sat_fix_msg.latitude  = nav_sat_fix_msg.latitude / 1e7;
  nav_sat_fix_msg.longitude = nav_sat_fix_msg.longitude / 1e7;

  gps_sensor_ptr_->updateData(nav_sat_fix_msg);
}

void PixhawkPlatform::px4BatteryCallback(const px4_msgs::msg::BatteryStatus::SharedPtr msg) {
  auto timestamp = this->get_clock()->now();

  sensor_msgs::msg::BatteryState battery_msg;
  battery_msg.header.stamp = timestamp;

  battery_msg.voltage         = msg->voltage_v;
  battery_msg.temperature     = msg->temperature;
  battery_msg.current         = msg->current_a;
  battery_msg.charge          = NAN;
  battery_msg.capacity        = msg->capacity;
  battery_msg.design_capacity = msg->design_capacity;
  battery_msg.percentage      = 1.0 - msg->remaining;
  // TODO: config file with battery settings
  battery_msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
  battery_msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
  battery_msg.power_supply_technology =
      sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
  battery_msg.present = msg->connected;
  // battery_msg.cell_voltage = msg->voltage_cell_v;
  battery_msg.cell_voltage     = {};
  battery_msg.cell_temperature = {};
  battery_msg.location         = '0';
  battery_msg.serial_number    = std::to_string(msg->serial_number);

  if (msg->warning >= 0) {
    RCLCPP_WARN_ONCE(this->get_logger(), "Battery warning #%d", msg->warning);
  }

  // if (msg->faults >= 0) {
  //   RCLCPP_ERROR_ONCE(this->get_logger(), "Battery error bitmask: %d",
  //   msg->faults);
  // }

  battery_sensor_ptr_->updateData(battery_msg);
}

bool PixhawkPlatform::getFlagSimulationMode() {
  // TODO: check if this is better than creating a variable to store the value
  return this->get_parameter("use_sim_time").as_bool();
}
