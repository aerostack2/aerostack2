// "Copyright [year] <Copyright Owner>"

#include "dji_matrice_platform.hpp"

#include <as2_core/utils/control_mode_utils.hpp>
#include <cmath>
#include <rclcpp/logging.hpp>

#include "dji_flight_controller.hpp"
#include "dji_subscriber.hpp"
#include "dji_telemetry.hpp"

DJIMatricePlatform::DJIMatricePlatform(int argc, char** argv)
    : as2::AerialPlatform() {
  declare_parameter<bool>("publish_camera", false);
  get_parameter<bool>("publish_camera", publish_camera_);

  declare_parameter<bool>("enable_advanced_sensing", false);
  get_parameter<bool>("enable_advanced_sensing", enable_advanced_sensing_);

  declare_parameter<bool>("enable_mop_channel", false);
  get_parameter<bool>("enable_mop_channel", enable_mop_channel_);

  // TODO: READ_PARAMS
  linux_env_ptr_ =
      std::make_shared<LinuxSetup>(argc, argv, enable_advanced_sensing_);
  static auto timer_commands_ = this->create_timer(
      std::chrono::milliseconds(30), [this]() { this->sendCommand(); });
}

void DJIMatricePlatform::configureSensors() {
  if (publish_camera_) {
    RCLCPP_INFO(get_logger(), "Main camera is enabled.");
    camera_handler_ = std::make_shared<DJICameraHandler>(vehicle_, this);
    camera_handler_->start_camera();
  }

  gimbal_handler_ = std::make_shared<DJIGimbalHandler>(vehicle_, this);
  camera_trigger_ = std::make_shared<DJICameraTrigger>(vehicle_, this);

  dji_subscriptions_.emplace_back(
      std::make_shared<DJISubscriptionOdometry>(this, vehicle_));
  dji_subscriptions_.emplace_back(
      std::make_shared<DJISubscriptionFlightStatus>(this, vehicle_));
  dji_subscriptions_.emplace_back(
      std::make_shared<DJISubscriptionBattery>(this, vehicle_));
  // dji_subscriptions_.emplace_back(
  //     std::make_shared<DJISubscriptionImu>(this, vehicle_));
  // FIXME: fix before using it on EKFs
  // dji_subscriptions_.emplace_back(
  //     std::make_shared<DJISubscriptionCompass>(this, vehicle_));
  // dji_subscriptions_.emplace_back(
  //     std::make_shared<DJISubscriptionRTK>(this, vehicle_));
};

void DJIMatricePlatform::printDJIError(ErrorCode::ErrorCodeType error) {
  // RCLCPP_WARN(this->get_logger(), "DJI ERROR Code Type : %ld", error);
  // auto error_msgs = ErrorCode::getErrorCode(ErrorCode::getModuleID(error),
  // ErrorCode::getFunctionID(error),ErrorCode::getRawRetCode(error));
  ErrorCode::printErrorCodeMsg(error);
}

bool DJIMatricePlatform::ownSetArmingState(bool state) {
  ErrorCode::ErrorCodeType error;
  if (state) {
    return true;
    error = vehicle_->flightController->turnOnMotorsSync(10);
  } else {
    error = vehicle_->flightController->turnOffMotorsSync(10);
    return true;
  }
  // return true;
  if (error) {
    RCLCPP_ERROR(get_logger(), "Failed to set arming state: %d", state);
    printDJIError(error);
    return false;
  }
  return true;
};

bool DJIMatricePlatform::ownTakeoff() {
  RCLCPP_INFO(this->get_logger(), "Taking off");
  int timeout = 10;
  ErrorCode::ErrorCodeType error =
      vehicle_->flightController->startTakeoffSync(timeout);

  // Wait timeout
  rclcpp::sleep_for(std::chrono::seconds(timeout));

  // RCLCPP_WARN(this->get_logger(),"ERROR CODE: %ld ", error);
  if (error) {
    printDJIError(error);
    RCLCPP_ERROR(this->get_logger(), "Takeoff Failed");
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "Takeoff Success");
  return true;
};

bool DJIMatricePlatform::ownLand() {
  RCLCPP_INFO(this->get_logger(), "Landing");
  control_flag_ = 0x00;
  ErrorCode::ErrorCodeType error =
      vehicle_->flightController->startLandingSync(10);
  // For hard landing :
  // ErrorCode::ErrorCodeType error =
  // vehicle_->flightController->startForceLandingSync(10);

  if (error) {
    printDJIError(error);
    RCLCPP_ERROR(this->get_logger(), "Landing Failed");
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "Landing Success");
  return true;
};

bool DJIMatricePlatform::ownSetOffboardControl(bool offboard) {
  auto _error =
      this->vehicle_->flightController->obtainJoystickCtrlAuthoritySync(10);
  printDJIError(_error);
  return true;
  ErrorCode::ErrorCodeType error;
  if (offboard) {
    error =
        this->vehicle_->flightController->obtainJoystickCtrlAuthoritySync(10);
    // ErrorCode::SUCCESS
  } else {
    RCLCPP_WARN(this->get_logger(), "Manual mode cannot be set from ");
  }
  if (error) {
    printDJIError(error);
    RCLCPP_ERROR(this->get_logger(), "Failed to set offboard control: %d",
                 offboard);
    ErrorCode::getErrorCodeMsg(error);
    return false;
  }
  return true;
};

bool DJIMatricePlatform::ownSetPlatformControlMode(
    const as2_msgs::msg::ControlMode& msg) {
  DJI::OSDK::FlightController::JoystickMode prov_mode;

  if (msg.control_mode == as2_msgs::msg::ControlMode::HOVER) {
    prov_mode.horizontalLogic =
        FlightController::HorizontalLogic::HORIZONTAL_VELOCITY;
    prov_mode.verticalLogic =
        FlightController::VerticalLogic::VERTICAL_VELOCITY;
    prov_mode.yawLogic = FlightController::YawLogic::YAW_RATE;
    prov_mode.stableMode = FlightController::StableMode::STABLE_ENABLE;
    prov_mode.horizontalCoordinate =
        FlightController::HorizontalCoordinate::HORIZONTAL_GROUND;
    dji_joystick_mode_ = prov_mode;
    return true;
  }

  // SET YAW_MODE
  if (msg.yaw_mode == as2_msgs::msg::ControlMode::YAW_ANGLE) {
    prov_mode.yawLogic = FlightController::YawLogic::YAW_ANGLE;
  } else if (msg.yaw_mode == as2_msgs::msg::ControlMode::YAW_SPEED) {
    prov_mode.yawLogic = FlightController::YawLogic::YAW_RATE;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Unknown yaw mode");
    return false;
  }

  switch (msg.control_mode) {
    case as2_msgs::msg::ControlMode::POSITION: {
      prov_mode.horizontalLogic =
          FlightController::HorizontalLogic::HORIZONTAL_POSITION;
      prov_mode.verticalLogic =
          FlightController::VerticalLogic::VERTICAL_POSITION;
    } break;
    case as2_msgs::msg::ControlMode::SPEED: {
      prov_mode.horizontalLogic =
          FlightController::HorizontalLogic::HORIZONTAL_VELOCITY;
      prov_mode.verticalLogic =
          FlightController::VerticalLogic::VERTICAL_VELOCITY;
    } break;
    case as2_msgs::msg::ControlMode::SPEED_IN_A_PLANE: {
      prov_mode.horizontalLogic =
          FlightController::HorizontalLogic::HORIZONTAL_VELOCITY;
      prov_mode.verticalLogic =
          FlightController::VerticalLogic::VERTICAL_POSITION;
    } break;
    case as2_msgs::msg::ControlMode::ATTITUDE: {
      prov_mode.horizontalLogic =
          FlightController::HorizontalLogic::HORIZONTAL_ANGLE;
      prov_mode.verticalLogic =
          FlightController::VerticalLogic::VERTICAL_THRUST;
    } break;
    case as2_msgs::msg::ControlMode::ACRO: {
      prov_mode.horizontalLogic =
          FlightController::HorizontalLogic::HORIZONTAL_ANGULAR_RATE;
      prov_mode.verticalLogic =
          FlightController::VerticalLogic::VERTICAL_THRUST;
    } break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown control mode");
      return false;
  }

  // ADD STABLE_MODE enabled
  prov_mode.stableMode = FlightController::StableMode::STABLE_ENABLE;
  // ADD HORIZONTAL_FRAME enabled
  prov_mode.horizontalCoordinate =
      FlightController::HorizontalCoordinate::HORIZONTAL_GROUND;
  dji_joystick_mode_ = prov_mode;

  return true;
};

bool DJIMatricePlatform::ownSendCommand() {
  // if (control_flag_ == 0x00) {
  //   // RCLCPP_ERROR(this->get_logger(), "Control flag is not set");
  //   return false;
  // }
  //
  //
  //
  // vehicle_->flightController
  // ->

  // vehicle_->control->obtainCtrlAuthority(1);
  // ownSetOffboardControl(true);
  // vehicle_->initControl();
  if (platform_info_msg_.current_control_mode.control_mode ==
      as2_msgs::msg::ControlMode::HOVER) {
    // send all zeros
    RCLCPP_INFO(this->get_logger(), " HOVERING");
    vehicle_->flightController->setJoystickMode(dji_joystick_mode_);
    FlightController::JoystickCommand joystick_cmd = {
        (float)0,
        (float)0,
        (float)0,
        (float)0,
    };
    vehicle_->flightController->setJoystickCommand(joystick_cmd);
    vehicle_->flightController->joystickAction();
    return true;
  }
  const auto clock = this->get_clock();
  // RCLCPP_INFO_THROTTLE(this->get_logger(), *clock, 1000,
  //                      "current control mode: %s",
  //                      as2::control_mode::controlModeToString(
  //                          platform_info_msg_.current_control_mode.control_mode)
  //                          .c_str());
  //
  if (!this->has_new_references_) {
    // RCLCPP_ERROR(this->get_logger(), "No new references");
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                          "No new references since mode change");
    return true;
  }

  double x, y, z, yaw;
  x = y = z = yaw = 0.0;

  if (platform_info_msg_.current_control_mode.yaw_mode ==
      as2_msgs::msg::ControlMode::YAW_ANGLE) {
    tf2::Quaternion q(this->command_pose_msg_.pose.orientation.x,
                      this->command_pose_msg_.pose.orientation.y,
                      this->command_pose_msg_.pose.orientation.z,
                      this->command_pose_msg_.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double _, __, yaw_;
    m.getRPY(_, __, yaw);
    yaw = -yaw * 180.0 / M_PI;
  } else if (platform_info_msg_.current_control_mode.yaw_mode ==
             as2_msgs::msg::ControlMode::YAW_SPEED) {
    yaw = -this->command_twist_msg_.twist.angular.z * 180.0 / M_PI;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Unknown yaw mode");
    return false;
  }

  switch (platform_info_msg_.current_control_mode.control_mode) {
    // case as2_msgs::msg::ControlMode::POSITION: {
    //   // DJI FRAME IS NEU
    //   x = this->command_pose_msg_.pose.position.y;
    //   y = -this->command_pose_msg_.pose.position.x;
    //   z = this->command_pose_msg_.pose.position.z;
    // } break;
    case as2_msgs::msg::ControlMode::SPEED: {
      // Conversion from AS2 ENU frame into DJI NEU frame
      x = this->command_twist_msg_.twist.linear.y;
      y = this->command_twist_msg_.twist.linear.x;
      z = this->command_twist_msg_.twist.linear.z;
    } break;
    // case as2_msgs::msg::ControlMode::SPEED_IN_A_PLANE: {
    //   x = this->command_twist_msg_.twist.linear.y;
    //   y = -this->command_twist_msg_.twist.linear.x;
    //   z = this->command_pose_msg_.pose.position.z;
    // } break;
    // case as2_msgs::msg::ControlMode::ATTITUDE: {
    //   // DJI FRAME IS FRU
    //   // obtaining rpy from quaternion
    //   tf2::Quaternion q(
    //       this->command_pose_msg_.pose.orientation.x,
    //       this->command_pose_msg_.pose.orientation.y,
    //       this->command_pose_msg_.pose.orientation.z,
    //       this->command_pose_msg_.pose.orientation.w);
    //   tf2::Matrix3x3 m(q);
    //   m.getRPY(x, y, yaw);
    //   // TODO: CHECK convert into degrees
    //   x = x * 180.0 / M_PI;
    //   y = -y * 180.0 / M_PI;  // DJI FRAME is FRU
    //   yaw = yaw * 180.0 / M_PI;
    //   z = this->command_thrust_msg_.thrust;
    // } break;
    // case as2_msgs::msg::ControlMode::ACRO: {
    //   // convert speeds from rad/s into deg/s
    //   x = this->command_twist_msg_.twist.angular.x;
    //   y = this->command_twist_msg_.twist.angular.y;
    //   yaw = this->command_twist_msg_.twist.angular.z;
    //   z = this->command_thrust_msg_.thrust;
    //   // TODO: CHECK convert into degrees/s
    //   x = x * 180.0 / M_PI;
    //   y = -y * 180.0 / M_PI;  // DJI FRAME is FRU
    //   yaw = yaw * 180.0 / M_PI;
    // } break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown control mode in send command");
      return false;
  }
  // Control::CtrlData ctrl_data(control_flag_, x, y, z, yaw);
  // vehicle_->control->flightCtrl(ctrl_data);

  // DJI::OSDK::FlightController::JoystickMode joystick_mode = {
  //     FlightController::HorizontalLogic::HORIZONTAL_VELOCITY,
  //     FlightController::VerticalLogic::VERTICAL_VELOCITY,
  //     FlightController::YawLogic::YAW_RATE,
  //     FlightController::HorizontalCoordinate::HORIZONTAL_GROUND,
  //     FlightController::StableMode::STABLE_ENABLE};

  // RCLCPP_INFO(this->get_logger(), "CMD commands = [ %.3f, %.3f, %.3f, %.3f
  // ]",
  //             x, y, z, yaw);

  vehicle_->flightController->setJoystickMode(dji_joystick_mode_);
  FlightController::JoystickCommand joystick_cmd = {
      (float)x,
      (float)y,
      (float)z,
      (float)yaw,
  };
  vehicle_->flightController->setJoystickCommand(joystick_cmd);
  vehicle_->flightController->joystickAction();
  return true;
}

int DJIMatricePlatform::djiInitVehicle() {
  vehicle_ = linux_env_ptr_->getVehicle();
  if (vehicle_ == NULL) {
    std::cout << "Vehicle not initialized, exiting.\n";
    return -1;
  }
  return 0;
};

int DJISubscription::n_packages_ = -1;
