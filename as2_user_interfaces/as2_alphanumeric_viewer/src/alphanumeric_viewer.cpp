// Copyright 2024 Universidad Politécnica de Madrid
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

/*!*******************************************************************************************
 *  \file       alphanumeric_viewer.cpp
 *  \brief      Alphanumeric viewer source file.
 *  \authors    Javier Melero Deza
 *  \copyright  Copyright (c) 2024 Universidad Politécnica de Madrid
 *              All Rights Reserved
 ********************************************************************************/

#include "alphanumeric_viewer.hpp"

AlphanumericViewer::AlphanumericViewer()
: as2::Node("alphanumeric_viewer") {}

void AlphanumericViewer::run()
{
  command = getch();
  switch (command) {
    case 'M':
    case 'm':  // Navigation
      erase();
      refresh();
      printSummaryMenu();
      window = 0;
      break;
    case 'S':
    case 's':  // Sensor
      erase();
      refresh();
      printSensorMenu();
      window = 1;
      break;
    case 'N':
    case 'n':  // Navigation
      erase();
      refresh();
      printNavigationMenu();
      window = 2;
      break;
    case 'P':
    case 'p':  // Navigation
      erase();
      refresh();
      printPlatformMenu();
      window = 3;
      break;
  }

  if (command == '\033') {
    getch();
    switch (getch()) {  // the real value
      case 'D':
        // code for arrow left
        if (window == 0) {
          break;
        }
        window--;
        break;
      case 'C':
        // code for arrow right
        if (window == 3) {
          break;
        }
        window++;
        break;
    }
    switch (window) {
      case 0:
        erase();
        refresh();
        printSummaryMenu();
        break;
      case 1:
        erase();
        refresh();
        printSensorMenu();
        break;
      case 2:
        erase();
        refresh();
        printNavigationMenu();
        break;
      case 3:
        erase();
        refresh();
        printPlatformMenu();
        break;
    }
  }

  // Print values
  switch (window) {
    case 0:
      printSummaryValues();
      break;
    case 1:
      printSensorValues();
      break;
    case 2:
      printNavigationValues();
      break;
    case 3:
      printPlatformValues();
      break;
  }
  // erase();
  // Refresh
  refresh();
}

void AlphanumericViewer::setupNode()
{
  interface_printout_stream << std::fixed << std::setprecision(2) << std::setfill('0');

  self_localization_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    this->generate_global_name(as2_names::topics::self_localization::pose),
    as2_names::topics::sensor_measurements::qos,
    std::bind(&AlphanumericViewer::poseCallback, this, std::placeholders::_1));

  self_localization_speed_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    this->generate_global_name(as2_names::topics::self_localization::twist),
    as2_names::topics::self_localization::qos,
    std::bind(&AlphanumericViewer::twistCallback, this, std::placeholders::_1));

  battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
    this->generate_global_name(as2_names::topics::sensor_measurements::battery),
    as2_names::topics::sensor_measurements::qos,
    std::bind(&AlphanumericViewer::batteryCallback, this, std::placeholders::_1));

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    this->generate_global_name(as2_names::topics::sensor_measurements::imu),
    as2_names::topics::sensor_measurements::qos,
    std::bind(&AlphanumericViewer::imuCallback, this, std::placeholders::_1));

  status_sub_ = this->create_subscription<as2_msgs::msg::PlatformInfo>(
    this->generate_global_name(as2_names::topics::platform::info),
    as2_names::topics::platform::qos,
    std::bind(&AlphanumericViewer::platformCallback, this, std::placeholders::_1));

  actuator_command_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    this->generate_global_name(as2_names::topics::actuator_command::pose),
    as2_names::topics::actuator_command::qos,
    std::bind(&AlphanumericViewer::actuatorPoseCallback, this, std::placeholders::_1));

  actuator_command_thrust_sub_ = this->create_subscription<as2_msgs::msg::Thrust>(
    this->generate_global_name(as2_names::topics::actuator_command::thrust),
    as2_names::topics::actuator_command::qos,
    std::bind(&AlphanumericViewer::actuatorThrustCallback, this, std::placeholders::_1));

  actuator_command_twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    this->generate_global_name(as2_names::topics::actuator_command::twist),
    as2_names::topics::actuator_command::qos,
    std::bind(&AlphanumericViewer::actuatorSpeedCallback, this, std::placeholders::_1));

  controller_info_sub_ = this->create_subscription<as2_msgs::msg::ControllerInfo>(
    this->generate_global_name(as2_names::topics::controller::info),
    as2_names::topics::controller::qos_info,
    std::bind(&AlphanumericViewer::controllerCallback, this, std::placeholders::_1));

  position_reference_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    this->generate_global_name(as2_names::topics::motion_reference::pose),
    as2_names::topics::motion_reference::qos,
    std::bind(&AlphanumericViewer::poseReferenceCallback, this, std::placeholders::_1));

  speed_reference_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    this->generate_global_name(as2_names::topics::motion_reference::twist),
    as2_names::topics::motion_reference::qos,
    std::bind(&AlphanumericViewer::speedReferenceCallback, this, std::placeholders::_1));

  /*trajectory_reference_sub_ =
     this->create_subscription<as2_msgs::msg::TrajectoryWaypoints::SharedPtr>(
      this->generate_global_name(as2_names::topics::motion_reference::trajectory),
      as2_names::topics::motion_reference::traj_gen_qos,
      std::bind(&AlphanumericViewer::trajectoryReferenceCallback, this, std::placeholders::_1));*/

  gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
    this->generate_global_name(as2_names::topics::sensor_measurements::gps),
    as2_names::topics::sensor_measurements::qos,
    std::bind(&AlphanumericViewer::gpsCallback, this, std::placeholders::_1));
}

void AlphanumericViewer::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr _msg)
{
  self_localization_pose_ = *_msg;
  current_pose_aux = true;
}
void AlphanumericViewer::twistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr _msg)
{
  self_localization_twist_ = *_msg;
  current_speed_aux = true;
}
void AlphanumericViewer::batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr _msg)
{
  battery_status_ = *_msg;
  if (battery_status_.percentage > 1.0) {
    battery_mode_ = 1;
  }
  battery_aux = true;
}
void AlphanumericViewer::imuCallback(const sensor_msgs::msg::Imu::SharedPtr _msg)
{
  imu_ = *_msg;
  imu_aux = true;
}
void AlphanumericViewer::platformCallback(const as2_msgs::msg::PlatformInfo::SharedPtr _msg)
{
  platform_info_ = *_msg;
  platform_info_aux = true;
}
void AlphanumericViewer::actuatorPoseCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr _msg)
{
  actuator_pose_ = *_msg;
  actuator_command_pose_aux = true;
}
void AlphanumericViewer::actuatorThrustCallback(const as2_msgs::msg::Thrust::SharedPtr _msg)
{
  actuator_thrust_ = *_msg;
  actuator_command_thrust_aux = true;
}
void AlphanumericViewer::actuatorSpeedCallback(
  const geometry_msgs::msg::TwistStamped::SharedPtr _msg)
{
  actuator_twist_ = *_msg;
  actuator_command_twist_aux = true;
}
void AlphanumericViewer::controllerCallback(const as2_msgs::msg::ControllerInfo::SharedPtr _msg)
{
  controller_info_ = *_msg;
  controller_info_aux = true;
}
void AlphanumericViewer::poseReferenceCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr _msg)
{
  reference_pose_ = *_msg;
  current_pose_reference_aux = true;
}
void AlphanumericViewer::speedReferenceCallback(
  const geometry_msgs::msg::TwistStamped::SharedPtr _msg)
{
  reference_twist_ = *_msg;
  current_speed_reference_aux = true;
}
/*void AlphanumericViewer::trajectoryReferenceCallback (const
as2_msgs::msg::TrajectoryWaypoints::SharedPtr _msg){ reference_traj_ = *_msg;
  current_trajectory_reference_aux = true;
}*/
void AlphanumericViewer::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr _msg)
{
  gps_ = *_msg;
  gps_aux = true;
}

void AlphanumericViewer::printSummaryMenu()
{
  clearValues();

  move(0, 0);
  printw("                - ALPHANUMERIC VIEWER OF AERIAL ROBOTICS DATA -");
  move(1, 0);
  printw("          Key: M (Summary), S (sensors), N (navigation), P (platform)");
  move(2, 0);
  printw("               ^                                                     ");
  // Left column
  move(3, 0);
  printw(" Drone id:");
  move(4, 0);
  printw(" Battery charge:");
  move(6, 0);
  printw(" IMU MEASUREMENTS");
  move(7, 0);
  printw(" Orientation IMU (ypr):");
  move(8, 0);
  printw(" Angular speed IMU (ypr):");
  move(9, 0);
  printw(" Acceleration IMU (xyz):");
  move(11, 0);
  printw(" LOCALIZATION");
  move(12, 0);
  printw(" Position (xyz):");
  move(13, 0);
  printw(" Linear Speed (xyz):");
  move(14, 0);
  printw(" Orientation (ypr):");
  move(15, 0);
  printw(" Angular Speed (ypr):");
  // right column
  move(6, 58);
  printw(" PLATFORM STATUS");
  move(8, 58);
  printw(" Conected:");
  move(9, 58);
  printw(" Armed:");
  move(10, 58);
  printw(" Offboard:");
  move(12, 58);
  printw(" Status:");
  // Bottom layout
  move(17, 0);
  printw(" CONTROLLER CONTROL MODE");
  move(18, 0);
  printw(" Yaw Mode:");
  move(19, 0);
  printw(" Control Mode:");
  move(20, 0);
  printw(" Frame Mode:");
  move(17, 41);
  printw(" PLATFORM CONTROL MODE");
  move(18, 41);
  printw(" Yaw Mode:");
  move(19, 41);
  printw(" Control Mode:");
  move(20, 41);
  printw(" Frame Mode:");
}

void AlphanumericViewer::printNavigationMenu()
{
  clearValues();

  move(0, 0);
  printw("                - ALPHANUMERIC VIEWER OF AERIAL ROBOTICS DATA -");
  move(1, 0);
  printw("          Key: M (Summary), S (sensors), N (navigation), P (platform)");
  move(2, 0);
  printw("                                         ^                           ");
  // Measurements
  move(4, 0);
  printw(" MEASUREMENTS");
  move(5, 0);
  printw(" Altitude(z):");
  move(6, 0);
  printw(" Orientation IMU (ypr):");
  move(7, 0);
  printw(" Angular speed IMU (ypr):");
  move(8, 0);
  printw(" Acceleration IMU (xyz):");

  // Localization
  move(11, 0);
  printw(" LOCALIZATION");
  move(12, 0);
  printw(" Position (xyz):");
  move(13, 0);
  printw(" Linear Speed (xyz):");
  move(14, 0);
  printw(" Orientation (ypr):");
  move(15, 0);
  printw(" Angular Speed (ypr):");
  // move(16,0);
  // printw(" Status:");
}

void AlphanumericViewer::printSensorMenu()
{
  clearValues();

  move(0, 0);
  printw("                - ALPHANUMERIC VIEWER OF AERIAL ROBOTICS DATA -");
  move(1, 0);
  printw("          Key: M (Summary), S (sensors), N (navigation), P (platform)");
  move(2, 0);
  printw("                            ^                                        ");
  // Left column
  move(3, 0);
  printw(" Drone id:");
  move(5, 0);
  printw(" Battery charge:");
  move(7, 0);
  printw(" Speed (x,y,z):");
  move(9, 0);
  printw(" Orientation IMU (yaw,pitch,roll):");
  move(11, 0);
  printw(" Angular Speed IMU (yaw,pitch,roll):");
  move(13, 0);
  printw(" Acceleration IMU (x,y,z):");

  // Right column
  move(3, 50);
  printw("Altitude (-z):");
  move(5, 50);
  printw("Altitude (sea level):");
  move(7, 50);
  printw("Temperature:");
  move(9, 50);
  printw("GPS Coordinates:");
  move(10, 50);
  printw("Lat:");
  move(11, 50);
  printw("Lon:");
  move(12, 50);
  printw("Alt:");
}

void AlphanumericViewer::printPlatformMenu()
{
  clearValues();

  move(0, 0);
  printw("                - ALPHANUMERIC VIEWER OF AERIAL ROBOTICS DATA -");
  move(1, 0);
  printw("          Key: M (Summary), S (sensors), N (navigation), P (platform)");
  move(2, 0);
  printw("                                                         ^           ");
  // Left column
  // Reference

  // Actuator commands
  move(4, 0);
  printw(" ACTUATOR COMMANDS");
  move(5, 0);
  printw(" Orientation (r,p,y):");
  move(6, 0);
  printw(" Linear Speed (x,y,z):");
  move(7, 0);
  printw(" Thrust:");
  move(8, 0);
  printw(" Angular Speed (r,p,y):");

  move(10, 0);
  printw(" REFERENCES");
  move(11, 0);
  printw(" Position (x,y,z):");
  move(12, 0);
  printw(" Linear Speed (x,y,z):");
  move(13, 0);
  printw(" Orientation (r,p,y):");
  move(14, 0);
  printw(" Angular Speed (r,p,y):");
  move(16, 0);
  printw(" CONTROLLER CONTROL MODE");
  move(17, 0);
  printw(" Yaw Mode:");
  move(18, 0);
  printw(" Control Mode:");
  move(19, 0);
  printw(" Frame Mode:");
  move(16, 41);
  printw(" PLATFORM CONTROL MODE");
  move(17, 41);
  printw(" Yaw Mode:");
  move(18, 41);
  printw(" Control Mode:");
  move(19, 41);
  printw(" Frame Mode:");
  move(5, 58);
  printw(" Conected:");
  move(6, 58);
  printw(" Armed:");
  move(7, 58);
  printw(" Offboard:");
  move(10, 58);
  printw(" Status:");
}

void AlphanumericViewer::printStream(float var, bool aux)
{
  if (aux) {
    interface_printout_stream.clear();
    interface_printout_stream.str(std::string());
    if (var > -0.01) {
      interface_printout_stream << std::setw(5) << std::internal << fabs(var);
      attron(COLOR_PAIR(1));
      printw(" %s", interface_printout_stream.str().c_str());
      attroff(COLOR_PAIR(1));
    } else {
      interface_printout_stream << std::setw(6) << std::internal << var;
      attron(COLOR_PAIR(2));
      printw("%s", interface_printout_stream.str().c_str());
      attroff(COLOR_PAIR(2));
    }
  } else {
    printw(" --.--");
  }
}

// Print float using stringstream with 3 units
void AlphanumericViewer::printStream3(float var, bool aux)
{
  if (aux) {
    interface_printout_stream.clear();
    interface_printout_stream.str(std::string());
    if (var > -0.01) {
      interface_printout_stream << std::setw(6) << std::internal << fabs(var);
      attron(COLOR_PAIR(1));
      printw(" %s", interface_printout_stream.str().c_str());
      attroff(COLOR_PAIR(1));
    } else {
      interface_printout_stream << std::setw(7) << std::internal << var;
      attron(COLOR_PAIR(2));
      printw("%s", interface_printout_stream.str().c_str());
      attroff(COLOR_PAIR(2));
    }
  } else {
    printw(" ---.--");
  }
}

// Print double using stringstream
void AlphanumericViewer::printStream(double var, bool aux)
{
  if (aux) {
    interface_printout_stream.clear();
    interface_printout_stream.str(std::string());
    if (var > -0.01) {
      interface_printout_stream << std::setw(5) << std::internal << fabs(var);
      attron(COLOR_PAIR(1));
      printw(" %s", interface_printout_stream.str().c_str());
      attroff(COLOR_PAIR(1));
    } else {
      interface_printout_stream << std::setw(6) << std::internal << var;
      attron(COLOR_PAIR(2));
      printw("%s", interface_printout_stream.str().c_str());
      attroff(COLOR_PAIR(2));
    }
  } else {
    printw(" --.--");
  }
}

void AlphanumericViewer::printSummaryValues()
{
  move(3, 11);
  attron(COLOR_PAIR(4));
  printw("%s", this->get_namespace());
  attroff(COLOR_PAIR(4));
  move(4, 17);
  printBattery();

  tf2::Matrix3x3 imu_m(tf2::Quaternion(
      imu_.orientation.x, imu_.orientation.y, imu_.orientation.z,
      imu_.orientation.w));
  double r = 0;
  double p = 0;
  double yaw = 0;
  imu_m.getRPY(r, p, yaw);
  if (std::isnan(r)) {r = 0.0;}
  if (std::isnan(p)) {p = 0.0;}
  if (std::isnan(yaw)) {yaw = 0.0;}

  move(7, 26);
  printStream(yaw, imu_aux);
  printw(",");
  move(7, 33);
  printStream(p, imu_aux);
  printw(",");
  move(7, 40);
  printStream(r, imu_aux);
  printw(" rad   ");
  move(8, 26);
  printStream(imu_.angular_velocity.z, imu_aux);
  printw(",");
  move(8, 33);
  printStream(imu_.angular_velocity.y, imu_aux);
  printw(",");
  move(8, 40);
  printStream(imu_.angular_velocity.x, imu_aux);
  printw(" rad/s  ");

  // Acceleration IMU
  move(9, 26);
  printStream(imu_.linear_acceleration.x, imu_aux);
  printw(",");
  move(9, 33);
  printStream(imu_.linear_acceleration.y, imu_aux);
  printw(",");
  move(9, 40);
  printStream(imu_.linear_acceleration.z, imu_aux);
  printw(" m/s2   ");

  // Localization
  // Pose
  move(12, 26);
  printStream3(self_localization_pose_.pose.position.x, current_pose_aux);
  printw(",");
  move(12, 34);
  printStream3(self_localization_pose_.pose.position.y, current_pose_aux);
  printw(",");
  move(12, 42);
  printStream3(self_localization_pose_.pose.position.z, current_pose_aux);
  printw(" m ");
  // Speed
  move(13, 26);
  printStream(self_localization_twist_.twist.linear.x, current_speed_aux);
  printw(",");
  move(13, 33);
  printStream(self_localization_twist_.twist.linear.y, current_speed_aux);
  printw(",");
  move(13, 40);
  printStream(self_localization_twist_.twist.linear.z, current_speed_aux);
  printw(" m/s ");
  // Pose(ypr)
  tf2::Matrix3x3 pose_m(tf2::Quaternion(
      self_localization_pose_.pose.orientation.x, self_localization_pose_.pose.orientation.y,
      self_localization_pose_.pose.orientation.z, self_localization_pose_.pose.orientation.w));
  pose_m.getRPY(r, p, yaw);
  if (std::isnan(yaw)) {yaw = 0.0;}
  if (std::isnan(r)) {r = 0.0;}
  if (std::isnan(p)) {p = 0.0;}
  move(14, 26);
  printStream(yaw, current_pose_aux);
  printw(",");
  move(14, 33);
  printStream(p, current_pose_aux);
  printw(",");
  move(14, 40);
  printStream(r, current_pose_aux);
  printw(" rad ");
  // Speed(ypr)
  move(15, 26);
  printStream(self_localization_twist_.twist.angular.z, current_speed_aux);
  printw(",");
  move(15, 33);
  printStream(self_localization_twist_.twist.angular.y, current_speed_aux);
  printw(",");
  move(15, 40);
  printStream(self_localization_twist_.twist.angular.x, current_speed_aux);
  printw(" rad/s ");
  move(18, 11);
  printControlModeInYaw();
  move(19, 15);
  printControlModeInControl();
  move(20, 13);
  printControlModeInFrame();
  move(18, 52);
  printControlModeOutYaw();
  move(19, 56);
  printControlModeOutControl();
  move(20, 54);
  printControlModeOutFrame();
  move(12, 68);
  printQuadrotorState();
  printPlatformStatus(8);
}

void AlphanumericViewer::printSensorValues()
{
  // DroneID
  move(4, 4);
  attron(COLOR_PAIR(4));
  printw("%s", this->get_namespace());
  attroff(COLOR_PAIR(4));
  // Battery
  move(6, 4);
  printBattery();
  // Speed
  move(8, 4);
  printStream(self_localization_twist_.twist.linear.x, current_speed_aux);
  printw(",");
  move(8, 11);
  printStream(self_localization_twist_.twist.linear.y, current_speed_aux);
  printw(",");
  move(8, 18);
  printStream(self_localization_twist_.twist.linear.z, current_speed_aux);
  printw(" m/s   ");

  // Pose IMU
  tf2::Matrix3x3 imu_m(tf2::Quaternion(
      imu_.orientation.x, imu_.orientation.y, imu_.orientation.z,
      imu_.orientation.w));
  double r = 0;
  double p = 0;
  double yaw = 0;
  imu_m.getRPY(r, p, yaw);
  if (std::isnan(r)) {r = 0.0;}
  if (std::isnan(p)) {p = 0.0;}
  if (std::isnan(yaw)) {yaw = 0.0;}

  move(10, 4);
  printStream(yaw, imu_aux);
  printw(",");
  move(10, 11);
  printStream(p, imu_aux);
  printw(",");
  move(10, 18);
  printStream(r, imu_aux);
  printw(" rad   ");

  // Speed IMU
  move(12, 4);
  printStream(imu_.angular_velocity.z, imu_aux);
  printw(",");
  move(12, 11);
  printStream(imu_.angular_velocity.y, imu_aux);
  printw(",");
  move(12, 18);
  printStream(imu_.angular_velocity.x, imu_aux);
  printw(" rad/s  ");

  // Acceleration IMU
  move(14, 4);
  printStream(imu_.linear_acceleration.x, imu_aux);
  printw(",");
  move(14, 11);
  printStream(imu_.linear_acceleration.y, imu_aux);
  printw(",");
  move(14, 18);
  printStream(imu_.linear_acceleration.z, imu_aux);
  printw(" m/s2   ");

  // Altitude
  move(4, 52);
  printStream(self_localization_pose_.pose.position.z, current_pose_aux);
  printw(" m");
  // Altitude sea level
  move(6, 52);
  printStream(0.0, altitude_sea_level_aux);
  printw(" m");
  // Temperature
  move(8, 52);
  printStream(0.0, temperature_aux);
  printw(" Degrees celsius");
  interface_printout_stream << std::fixed << std::setprecision(8) << std::setfill('0');
  move(10, 54);
  printStream(gps_.latitude, gps_aux);
  move(11, 54);
  printStream(gps_.longitude, gps_aux);
  move(12, 54);
  printStream(gps_.altitude, gps_aux);
  interface_printout_stream << std::fixed << std::setprecision(2) << std::setfill('0');
}

void AlphanumericViewer::printNavigationValues()
{
  // Measurements
  move(5, 26);
  printStream(self_localization_pose_.pose.position.z, current_pose_aux);
  printw(" m");

  // Pose IMU
  tf2::Matrix3x3 imu_m(tf2::Quaternion(
      imu_.orientation.x, imu_.orientation.y, imu_.orientation.z,
      imu_.orientation.w));
  double r = 0;
  double p = 0;
  double yaw = 0;
  imu_m.getRPY(r, p, yaw);
  if (std::isnan(r)) {r = 0.0;}
  if (std::isnan(p)) {p = 0.0;}
  if (std::isnan(yaw)) {yaw = 0.0;}

  move(6, 26);
  printStream(yaw, imu_aux);
  printw(",");
  move(6, 33);
  printStream(p, imu_aux);
  printw(",");
  move(6, 40);
  printStream(r, imu_aux);
  printw(" rad   ");

  // Speed IMU
  move(7, 26);
  printStream(imu_.angular_velocity.z, imu_aux);
  printw(",");
  move(7, 33);
  printStream(imu_.angular_velocity.y, imu_aux);
  printw(",");
  move(7, 40);
  printStream(imu_.angular_velocity.x, imu_aux);
  printw(" rad/s  ");

  // Acceleration IMU
  move(8, 26);
  printStream(imu_.linear_acceleration.x, imu_aux);
  printw(",");
  move(8, 33);
  printStream(imu_.linear_acceleration.y, imu_aux);
  printw(",");
  move(8, 40);
  printStream(imu_.linear_acceleration.z, imu_aux);
  printw(" m/s2   ");

  // Localization
  // Pose
  move(12, 26);
  printStream3(self_localization_pose_.pose.position.x, current_pose_aux);
  printw(",");
  move(12, 34);
  printStream3(self_localization_pose_.pose.position.y, current_pose_aux);
  printw(",");
  move(12, 42);
  printStream3(self_localization_pose_.pose.position.z, current_pose_aux);
  printw(" m ");
  // Speed
  move(13, 26);
  printStream(self_localization_twist_.twist.linear.x, current_speed_aux);
  printw(",");
  move(13, 33);
  printStream(self_localization_twist_.twist.linear.y, current_speed_aux);
  printw(",");
  move(13, 40);
  printStream(self_localization_twist_.twist.linear.z, current_speed_aux);
  printw(" m/s ");
  // Pose(ypr)
  tf2::Matrix3x3 pose_m(tf2::Quaternion(
      self_localization_pose_.pose.orientation.x, self_localization_pose_.pose.orientation.y,
      self_localization_pose_.pose.orientation.z, self_localization_pose_.pose.orientation.w));
  pose_m.getRPY(r, p, yaw);
  if (std::isnan(yaw)) {yaw = 0.0;}
  if (std::isnan(r)) {r = 0.0;}
  if (std::isnan(p)) {p = 0.0;}
  move(14, 26);
  printStream(yaw, current_pose_aux);
  printw(",");
  move(14, 33);
  printStream(p, current_pose_aux);
  printw(",");
  move(14, 40);
  printStream(r, current_pose_aux);
  printw(" rad ");
  // Speed(ypr)
  move(15, 26);
  printStream(self_localization_twist_.twist.angular.z, current_speed_aux);
  printw(",");
  move(15, 33);
  printStream(self_localization_twist_.twist.angular.y, current_speed_aux);
  printw(",");
  move(15, 40);
  printStream(self_localization_twist_.twist.angular.x, current_speed_aux);
  printw(" rad/s ");
}

void AlphanumericViewer::printPlatformValues()
{
  // move(3,54);
  // printQuadrotorState();

  // Actuator commands
  if (thrust_aux) {
    move(5, 30);
    printStream(actuator_thrust_.thrust, thrust_aux);
    printw(" N ,");
    move(6, 37);
    printStream(actuator_thrust_.thrust_normalized, thrust_aux);
    printw(" normalized  ");
    // Speed(z)
    move(7, 30);
    printStream(actuator_twist_.twist.linear.z, actuator_command_twist_aux);
    printw(" m/s  ");
    // Thrust
    // Speed(yaw)
    move(8, 30);
    printStream(actuator_twist_.twist.angular.z, actuator_command_twist_aux);
    printw(" rad/s  ");
  } else {
    // Pitch roll
    tf2::Matrix3x3 actuator_m(
      tf2::Quaternion(
        actuator_pose_.pose.orientation.x, actuator_pose_.pose.orientation.y,
        actuator_pose_.pose.orientation.z, actuator_pose_.pose.orientation.w));
    double r = 0;
    double p = 0;
    double yaw = 0;
    actuator_m.getRPY(r, p, yaw);
    if (std::isnan(r)) {r = 0.0;}
    if (std::isnan(p)) {p = 0.0;}
    if (std::isnan(yaw)) {yaw = 0.0;}
    move(5, 28);
    printStream(r, current_pose_aux);
    printw(",");
    move(5, 35);
    printStream(p, current_pose_aux);
    printw(",");
    move(5, 42);
    printStream(yaw, current_pose_aux);
    printw(" rad  ");
    // Speed(z)
    move(6, 28);
    printStream(actuator_twist_.twist.linear.x, current_speed_aux);
    printw(",");
    move(6, 35);
    printStream(actuator_twist_.twist.linear.y, current_speed_aux);
    printw(",");
    move(6, 42);
    printStream(actuator_twist_.twist.linear.z, current_speed_aux);
    printw(" m/s  ");
    // Thrust
    move(7, 28);
    printStream(actuator_thrust_.thrust, thrust_aux);
    printw(" N  ");
    // Speed(yaw)
    move(8, 28);
    printStream(actuator_twist_.twist.angular.x, current_speed_aux);
    printw(",");
    move(8, 35);
    printStream(actuator_twist_.twist.angular.y, current_speed_aux);
    printw(",");
    move(8, 42);
    printStream(actuator_twist_.twist.angular.z, current_speed_aux);
    printw(" rad/s  ");
  }

  // References
  // Pose
  move(11, 28);
  printStream3(reference_pose_.pose.position.x, current_pose_reference_aux);
  printw(",");
  move(11, 36);
  printStream3(reference_pose_.pose.position.y, current_pose_reference_aux);
  printw(",");
  move(11, 44);
  printStream3(reference_pose_.pose.position.z, current_pose_reference_aux);
  printw(" m ");
  // Speed
  move(12, 28);
  printStream(reference_twist_.twist.linear.x, current_speed_reference_aux);
  printw(",");
  move(12, 35);
  printStream(reference_twist_.twist.linear.y, current_speed_reference_aux);
  printw(",");
  move(12, 42);
  printStream(reference_twist_.twist.linear.z, current_speed_reference_aux);
  printw(" m/s ");
  // Pose (yaw)
  tf2::Matrix3x3 pose_ref_m(
    tf2::Quaternion(
      reference_pose_.pose.orientation.x, reference_pose_.pose.orientation.y,
      reference_pose_.pose.orientation.z, reference_pose_.pose.orientation.w));
  double r = 0;
  double p = 0;
  double yaw = 0;
  pose_ref_m.getRPY(r, p, yaw);
  if (std::isnan(r)) {r = 0.0;}
  if (std::isnan(p)) {p = 0.0;}
  if (std::isnan(yaw)) {yaw = 0.0;}
  move(13, 28);
  printStream(r, current_pose_reference_aux);
  printw(",");
  move(13, 35);
  printStream(p, current_pose_reference_aux);
  printw(",");
  move(13, 42);
  printStream(yaw, current_pose_reference_aux);
  printw(" rad");
  // Speed (yaw)
  move(14, 28);
  printStream(reference_twist_.twist.angular.x, current_speed_reference_aux);
  printw(",");
  move(14, 35);
  printStream(reference_twist_.twist.angular.y, current_speed_reference_aux);
  printw(",");
  move(14, 42);
  printStream(reference_twist_.twist.angular.z, current_speed_reference_aux);
  printw(" rad/s  ");
  // Control mode
  move(17, 11);
  printControlModeInYaw();
  move(18, 15);
  printControlModeInControl();
  move(19, 13);
  printControlModeInFrame();
  move(17, 52);
  printControlModeOutYaw();
  move(18, 56);
  printControlModeOutControl();
  move(19, 54);
  printControlModeOutFrame();
  printPlatformStatus(5);
  move(10, 68);
  printQuadrotorState();
}

void AlphanumericViewer::printBattery()
{
  if (battery_aux) {
    interface_printout_stream << std::fixed << std::setprecision(0) << std::setfill(' ');
    interface_printout_stream.clear();
    interface_printout_stream.str(std::string());
    clrtoeol();
    refresh();
    float percentage = battery_status_.percentage;
    if (battery_mode_ == 0) {
      percentage = percentage * 100;
    }
    interface_printout_stream << std::setw(2) << std::internal << percentage;
    if (percentage == 100) {
      attron(COLOR_PAIR(1));
      printw(" %s", interface_printout_stream.str().c_str());
      attroff(COLOR_PAIR(1));
    }
    if (percentage > 50 && percentage < 100) {
      attron(COLOR_PAIR(1));
      printw(" %s", interface_printout_stream.str().c_str());
      attroff(COLOR_PAIR(1));
    }
    if (percentage <= 50 && percentage > 20) {
      attron(COLOR_PAIR(3));
      printw(" %s", interface_printout_stream.str().c_str());
      attroff(COLOR_PAIR(3));
    }
    if (percentage <= 20) {
      attron(COLOR_PAIR(2));
      printw(" %s", interface_printout_stream.str().c_str());
      attroff(COLOR_PAIR(2));
    }
  } else {  // Battery has not been received
    printw("---");
  }
  printw(" %%");
  interface_printout_stream << std::fixed << std::setprecision(2) << std::setfill('0');
}

void AlphanumericViewer::printPlatformStatus(int line)
{
  move(line, 70);
  if (platform_info_.connected) {
    printw("True ");
  } else {
    printw("False");
  }
  move(line + 1, 70);
  if (platform_info_.armed) {
    printw("True ");
  } else {
    printw("False");
  }
  move(line + 2, 70);
  if (platform_info_.offboard) {
    printw("True ");
  } else {
    printw("False");
  }
}

void AlphanumericViewer::printQuadrotorState()
{
  switch (platform_info_.status.state) {
    case as2_msgs::msg::PlatformStatus::LANDED:
      printw("LANDED    ");
      break;
    case as2_msgs::msg::PlatformStatus::FLYING:
      printw("FLYING    ");
      break;
    case as2_msgs::msg::PlatformStatus::EMERGENCY:
      printw("EMERGENCY ");
      break;
    case as2_msgs::msg::PlatformStatus::DISARMED:
      printw("DISARMED");
      break;
    case as2_msgs::msg::PlatformStatus::TAKING_OFF:
      printw("TAKING OFF");
      break;
    case as2_msgs::msg::PlatformStatus::LANDING:
      printw("LANDING   ");
      break;
  }
}
void AlphanumericViewer::printControlModeInYaw()
{
  switch (controller_info_.input_control_mode.yaw_mode) {
    case as2_msgs::msg::ControlMode::NONE:
      printw("NONE        ");
      break;
    case as2_msgs::msg::ControlMode::YAW_ANGLE:
      printw("YAW_ANGLE   ");
      break;
    case as2_msgs::msg::ControlMode::YAW_SPEED:
      printw("YAW_SPEED   ");
      break;
    default:
      printw("UNKNOWN     ");
      break;
  }
}

void AlphanumericViewer::printControlModeInControl()
{
  switch (controller_info_.input_control_mode.control_mode) {
    case as2_msgs::msg::ControlMode::UNSET:
      printw("UNSET        ");
      break;
    case as2_msgs::msg::ControlMode::HOVER:
      printw("HOVER        ");
      break;
    case as2_msgs::msg::ControlMode::POSITION:
      printw("POSITION     ");
      break;
    case as2_msgs::msg::ControlMode::SPEED:
      printw("SPEED        ");
      break;
    case as2_msgs::msg::ControlMode::SPEED_IN_A_PLANE:
      printw("SPEEDINAPLANE");
      break;
    case as2_msgs::msg::ControlMode::ATTITUDE:
      printw("ATTITUDE     ");
      break;
    case as2_msgs::msg::ControlMode::ACRO:
      printw("ACRO         ");
      break;
    case as2_msgs::msg::ControlMode::TRAJECTORY:
      printw("TRAJECTORY   ");
      break;
    case as2_msgs::msg::ControlMode::ACEL:
      printw("ACEL         ");
      break;
    default:
      printw("UNKNOWN      ");
      break;
  }
}

void AlphanumericViewer::printControlModeInFrame()
{
  switch (controller_info_.input_control_mode.reference_frame) {
    case as2_msgs::msg::ControlMode::UNDEFINED_FRAME:
      printw("UNDEFINED_FRAME     ");
      break;
    case as2_msgs::msg::ControlMode::LOCAL_ENU_FRAME:
      printw("LOCAL_ENU_FRAME     ");
      break;
    case as2_msgs::msg::ControlMode::BODY_FLU_FRAME:
      printw("BODY_FLU_FRAME      ");
      break;
    case as2_msgs::msg::ControlMode::GLOBAL_LAT_LONG_ASML:
      printw("GLOBAL_LAT_LONG_ASML");
      break;
    default:
      printw("UNKNOWN      ");
      break;
  }
}

void AlphanumericViewer::printControlModeOutYaw()
{
  switch (platform_info_.current_control_mode.yaw_mode) {
    case as2_msgs::msg::ControlMode::NONE:
      printw("NONE        ");
      break;
    case as2_msgs::msg::ControlMode::YAW_ANGLE:
      printw("YAW_ANGLE    ");
      break;
    case as2_msgs::msg::ControlMode::YAW_SPEED:
      printw("YAW_SPEED     ");
      break;
    default:
      printw("UNKNOWN      ");
      break;
  }
}

void AlphanumericViewer::printControlModeOutControl()
{
  switch (platform_info_.current_control_mode.control_mode) {
    case as2_msgs::msg::ControlMode::UNSET:
      printw("UNSET        ");
      break;
    case as2_msgs::msg::ControlMode::HOVER:
      printw("HOVER        ");
      break;
    case as2_msgs::msg::ControlMode::POSITION:
      printw("POSITION     ");
      break;
    case as2_msgs::msg::ControlMode::SPEED:
      printw("SPEED        ");
      break;
    case as2_msgs::msg::ControlMode::SPEED_IN_A_PLANE:
      printw("SPEEDINAPLANE");
      break;
    case as2_msgs::msg::ControlMode::ATTITUDE:
      printw("ATTITUDE     ");
      break;
    case as2_msgs::msg::ControlMode::ACRO:
      printw("ACRO         ");
      break;
    case as2_msgs::msg::ControlMode::TRAJECTORY:
      printw("TRAJECTORY   ");
      break;
    case as2_msgs::msg::ControlMode::ACEL:
      printw("ACEL         ");
      break;
    default:
      printw("UNKNOWN      ");
      break;
  }
}

void AlphanumericViewer::printControlModeOutFrame()
{
  switch (platform_info_.current_control_mode.reference_frame) {
    case as2_msgs::msg::ControlMode::UNDEFINED_FRAME:
      printw("UNDEFINED_FRAME     ");
      break;
    case as2_msgs::msg::ControlMode::LOCAL_ENU_FRAME:
      printw("LOCAL_ENU_FRAME     ");
      break;
    case as2_msgs::msg::ControlMode::BODY_FLU_FRAME:
      printw("BODY_FLU_FRAME      ");
      break;
    case as2_msgs::msg::ControlMode::GLOBAL_LAT_LONG_ASML:
      printw("GLOBAL_LAT_LONG_ASML");
      break;
    default:
      printw("UNKNOWN      ");
      break;
  }
}

void AlphanumericViewer::clearValues()
{
  gps_aux = false;
  imu_aux = false;
  thrust_aux = false;
  battery_aux = false;
  altitude_aux = false;
  temperature_aux = false;
  current_pose_aux = false;
  ground_speed_aux = false;
  current_speed_aux = false;
  platform_info_aux = false;
  controller_info_aux = false;
  altitude_sea_level_aux = false;
  actuator_command_pose_aux = false;
  current_pose_reference_aux = false;
  actuator_command_twist_aux = false;
  current_speed_reference_aux = false;
  actuator_command_thrust_aux = false;
  current_trajectory_reference_aux = false;
}

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturn AlphanumericViewer::on_configure(const rclcpp_lifecycle::State & _state)
{
  // Set subscriptions, publishers, services, actions, etc. here.
  setupNode();
  initscr();
  start_color();
  use_default_colors();
  curs_set(0);
  noecho();
  nodelay(stdscr, TRUE);
  erase();
  refresh();
  init_pair(1, COLOR_GREEN, -1);
  init_pair(2, COLOR_RED, -1);
  init_pair(3, COLOR_YELLOW, -1);
  init_pair(4, COLOR_CYAN, -1);

  printSummaryMenu();
  return CallbackReturn::SUCCESS;
}

CallbackReturn AlphanumericViewer::on_deactivate(const rclcpp_lifecycle::State & _state)
{
  // Clean up subscriptions, publishers, services, actions, etc. here.
  return CallbackReturn::SUCCESS;
}

CallbackReturn AlphanumericViewer::on_shutdown(const rclcpp_lifecycle::State & _state)
{
  // Clean other resources here.
  endwin();
  return CallbackReturn::SUCCESS;
}
