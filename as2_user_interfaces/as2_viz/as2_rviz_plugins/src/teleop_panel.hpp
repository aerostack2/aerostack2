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

#ifndef TELEOP_PANEL_HPP_
#define TELEOP_PANEL_HPP_

#ifndef Q_MOC_RUN
#include <functional>
#include <future>
#include <memory>
#include <string>

#include "as2_core/names/actions.hpp"
#include "as2_core/names/services.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_core/node.hpp"
#include "as2_motion_reference_handlers/hover_motion.hpp"
#include "as2_msgs/action/land.hpp"
#include "as2_msgs/action/takeoff.hpp"
#include "as2_msgs/msg/alert_event.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#endif

class QLineEdit;
class QPushButton;

namespace as2_rviz_plugins
{

// Here we declare our new subclass of rviz_common::Panel.  Every panel which
// can be added via the Panels/Add_New_Panel menu is a subclass of
// rviz_common::Panel.
class TeleopPanel : public rviz_common::Panel
{
  // This class uses Qt slots and is a subclass of QObject, so it needs
  // the Q_OBJECT macro.
  Q_OBJECT

public:
  // QWidget subclass constructors usually take a parent widget
  // parameter (which usually defaults to 0).  At the same time,
  // pluginlib::ClassLoader creates instances by calling the default
  // constructor (with no arguments).  Taking the parameter and giving
  // a default of 0 lets the default constructor work and also lets
  // someone using the class for something else to pass in a parent
  // widget as they normally would with Qt.
  explicit TeleopPanel(QWidget * parent = 0);

  // Now we declare overrides of rviz_common::Panel functions for saving and
  // loading data from the config file.  Here the data is the
  // topic name.
  virtual void load(const rviz_common::Config & config);
  virtual void save(rviz_common::Config config) const;

public Q_SLOTS:
  void takeoff();
  void land();
  void disarm(
    const rclcpp_action::ClientGoalHandle<
      as2_msgs::action::Land>::WrappedResult & result);
  void hover();
  void kill();

  // ros2 topic pub /drone0/alert_event as2_msgs/msg/AlertEvent "alert: -1
  // description: ''"

  // reads the namespace from the drone_editor_ and creates clients
  void updateDroneNs();

protected:
  QLineEdit * drone_editor_;
  QPushButton * takeoff_button_;
  QPushButton * land_button_;
  QPushButton * hover_button_;
  QPushButton * kill_button_;

  QString drone_namespace_;

  std::shared_ptr<as2::Node> node_;
  std::shared_ptr<rclcpp::Node> node2_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr arming_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr offboard_client_;
  rclcpp_action::Client<as2_msgs::action::Takeoff>::SharedPtr takeoff_client_;
  rclcpp_action::Client<as2_msgs::action::Land>::SharedPtr land_client_;
  rclcpp::Publisher<as2_msgs::msg::AlertEvent>::SharedPtr alert_pub_;
  std::shared_ptr<as2::motionReferenceHandlers::HoverMotion> hover_handler_;
};

}  // namespace as2_rviz_plugins

#endif  // TELEOP_PANEL_HPP_
