// Copyright (c) 2011, Willow Garage, Inc.
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Willow Garage, Inc. nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "teleop_panel.hpp"

#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPainter>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>
#include <stdio.h>

#include <memory>

#include "pluginlib/class_list_macros.hpp"

namespace as2_rviz_plugins
{

// We start with the constructor, doing the standard Qt thing of
// passing the optional *parent* argument on to the superclass
// constructor, and also zero-ing the velocities we will be
// publishing.
TeleopPanel::TeleopPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  QHBoxLayout * drone_layout = new QHBoxLayout;
  drone_layout->addWidget(new QLabel("Drone ns:"));
  drone_editor_ = new QLineEdit;
  drone_layout->addWidget(drone_editor_);

  // Lay out the topic field above the control widget.
  QVBoxLayout * layout = new QVBoxLayout;
  layout->addLayout(drone_layout);
  takeoff_button_ = new QPushButton("Takeoff");
  land_button_ = new QPushButton("Land");
  hover_button_ = new QPushButton("Hover");
  kill_button_ = new QPushButton("Kill");
  layout->addWidget(takeoff_button_);
  layout->addWidget(land_button_);
  layout->addWidget(hover_button_);
  layout->addWidget(kill_button_);
  setLayout(layout);

  // Next we make signal/slot connections.
  connect(
    drone_editor_, SIGNAL(editingFinished()), this,
    SLOT(updateDroneNs()));
  connect(takeoff_button_, SIGNAL(clicked()), this, SLOT(takeoff()));
  connect(land_button_, SIGNAL(clicked()), this, SLOT(land()));
  connect(hover_button_, SIGNAL(clicked()), this, SLOT(hover()));
  connect(kill_button_, SIGNAL(clicked()), this, SLOT(kill()));

  // Make the control widget start disabled
  takeoff_button_->setEnabled(false);

  node_ = std::make_shared<as2::Node>("teleop_panel_node");
  node2_ = std::make_shared<rclcpp::Node>("teleop_panel_node2");
}

// TODO(pariaspe): takeoff may be started before services finished which will cause
// takeoff failures. Clicking twice will make the drone takeoff
void TeleopPanel::takeoff()
{
  if (!arming_client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Arming service not available after waiting");
    return;
  }

  if (!offboard_client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Offboard service not available after waiting");
    return;
  }

  if (!takeoff_client_->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Action server not available after waiting");
    return;
  }

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = true;
  arming_client_->async_send_request(request);
  offboard_client_->async_send_request(request);

  auto goal_msg = as2_msgs::action::Takeoff::Goal();
  goal_msg.takeoff_height = 1.0;
  goal_msg.takeoff_speed = 1.0;
  RCLCPP_INFO(node_->get_logger(), "Taking off...");
  takeoff_client_->async_send_goal(goal_msg);
}

void TeleopPanel::land()
{
  if (!land_client_->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Action server not available after waiting");
    return;
  }

  auto goal_options =
    rclcpp_action::Client<as2_msgs::action::Land>::SendGoalOptions();
  // TODO(pariaspe): not building
  // goal_options.result_callback =
  //     std::bind(&TeleopPanel::disarm, node2_, std::placeholders::_1);

  auto goal_msg = as2_msgs::action::Land::Goal();
  goal_msg.land_speed = 1.0;

  land_client_->async_send_goal(goal_msg, goal_options);
}

void TeleopPanel::disarm(
  const rclcpp_action::ClientGoalHandle<as2_msgs::action::Land>::WrappedResult
  & result)
{
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_ERROR(node_->get_logger(), "Landing failed");
    return;
  }

  if (!offboard_client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Offboard service not available after waiting");
    return;
  }

  if (!arming_client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Arming service not available after waiting");
    return;
  }
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = false;
  offboard_client_->async_send_request(request);
  arming_client_->async_send_request(request);
}

void TeleopPanel::hover() {hover_handler_->sendHover();}

void TeleopPanel::kill()
{
  auto alert_msg = as2_msgs::msg::AlertEvent();
  alert_msg.alert = as2_msgs::msg::AlertEvent::KILL_SWITCH;
  alert_msg.description = "Manual kill from rviz_teleop_plugin";
  alert_pub_->publish(alert_msg);
}

// This is connected to QLineEdit::editingFinished() which
// fires when the user presses Enter or Tab or otherwise moves focus
// away.
void TeleopPanel::updateDroneNs()
{
  QString new_ns = drone_editor_->text();
  // Only take action if the name has changed.
  if (new_ns != drone_namespace_) {
    drone_namespace_ = new_ns;

    // Reset currect clients and action_client
    if (arming_client_ != NULL) {
      arming_client_.reset();
    }
    if (offboard_client_ != NULL) {
      offboard_client_.reset();
    }
    if (takeoff_client_ != NULL) {
      takeoff_client_.reset();
    }

    arming_client_ = node_->create_client<std_srvs::srv::SetBool>(
      drone_namespace_.toStdString() + "/" +
      as2_names::services::platform::set_arming_state);
    offboard_client_ = node_->create_client<std_srvs::srv::SetBool>(
      drone_namespace_.toStdString() + "/" +
      as2_names::services::platform::set_offboard_mode);
    takeoff_client_ = rclcpp_action::create_client<as2_msgs::action::Takeoff>(
      node_, drone_namespace_.toStdString() + "/" +
      as2_names::actions::behaviors::takeoff);
    land_client_ = rclcpp_action::create_client<as2_msgs::action::Land>(
      node_, drone_namespace_.toStdString() + "/" +
      as2_names::actions::behaviors::land);
    alert_pub_ = node_->create_publisher<as2_msgs::msg::AlertEvent>(
      drone_namespace_.toStdString() + "/" +
      as2_names::topics::global::alert_event,
      as2_names::topics::global::qos);
    hover_handler_ =
      std::make_shared<as2::motionReferenceHandlers::HoverMotion>(
      node_.get(),
      drone_namespace_.toStdString());       // Be careful, from shared_ptr to
                                             // ptr, what happens when the
                                             // shared_ptr is destroyed? BUM!

    // rviz_common::Panel defines the configChanged() signal.  Emitting it
    // tells RViz that something in this panel has changed that will
    // affect a saved config file.  Ultimately this signal can cause
    // QWidget::setWindowModified(true) to be called on the top-level
    // rviz_common::VisualizationFrame, which causes a little asterisk ("*")
    // to show in the window's title bar indicating unsaved changes.
    Q_EMIT configChanged();
  }

  takeoff_button_->setEnabled(true);
}

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void TeleopPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("drone_ns", drone_namespace_);
}

// Load all configuration data for this panel from the given Config object.
void TeleopPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  QString ns;
  if (config.mapGetString("drone_ns", &ns)) {
    drone_editor_->setText(ns);
    updateDroneNs();
  }
}

}  // namespace as2_rviz_plugins

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
PLUGINLIB_EXPORT_CLASS(as2_rviz_plugins::TeleopPanel, rviz_common::Panel)
