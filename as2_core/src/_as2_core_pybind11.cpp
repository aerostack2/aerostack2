// Copyright 2023 Universidad Politécnica de Madrid
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

#include <pybind11/pybind11.h>
#include "as2_core/names/actions.hpp"
#include "as2_core/names/services.hpp"
#include "as2_core/names/topics.hpp"

PYBIND11_MODULE(as2_names, m) {
  // TOPICS
  auto m_topics = m.def_submodule("topics", "Topics name defaults");

  auto m_global = m_topics.def_submodule("global");
  m_global.attr("alert_event") = as2_names::topics::global::alert_event;

  auto m_sensor_measurements = m_topics.def_submodule("sensor_measurements");
  m_sensor_measurements.attr("imu") = as2_names::topics::sensor_measurements::imu;
  m_sensor_measurements.attr("lidar") = as2_names::topics::sensor_measurements::lidar;
  m_sensor_measurements.attr("gps") = as2_names::topics::sensor_measurements::gps;
  m_sensor_measurements.attr("camera") = as2_names::topics::sensor_measurements::camera;
  m_sensor_measurements.attr("battery") = as2_names::topics::sensor_measurements::battery;
  m_sensor_measurements.attr("odom") = as2_names::topics::sensor_measurements::odom;

  auto m_ground_truth = m_topics.def_submodule("ground_truth");
  m_ground_truth.attr("pose") = as2_names::topics::ground_truth::pose;
  m_ground_truth.attr("twist") = as2_names::topics::ground_truth::twist;

  auto m_self_localization = m_topics.def_submodule("self_localization");
  m_self_localization.attr("odom") = as2_names::topics::self_localization::odom;
  m_self_localization.attr("pose") = as2_names::topics::self_localization::pose;
  m_self_localization.attr("twist") = as2_names::topics::self_localization::twist;

  auto m_motion_reference = m_topics.def_submodule("motion_reference");
  m_motion_reference.attr("pose") = as2_names::topics::motion_reference::pose;
  m_motion_reference.attr("twist") = as2_names::topics::motion_reference::twist;
  m_motion_reference.attr("trajectory") = as2_names::topics::motion_reference::trajectory;
  m_motion_reference.attr("modify_waypoint") = as2_names::topics::motion_reference::modify_waypoint;
  m_motion_reference.attr("traj_gen_info") = as2_names::topics::motion_reference::traj_gen_info;

  auto m_actuator_command = m_topics.def_submodule("actuator_command");
  m_actuator_command.attr("pose") = as2_names::topics::actuator_command::pose;
  m_actuator_command.attr("twist") = as2_names::topics::actuator_command::twist;
  m_actuator_command.attr("thrust") = as2_names::topics::actuator_command::thrust;
  m_actuator_command.attr("trajectory") = as2_names::topics::actuator_command::trajectory;

  auto m_platform = m_topics.def_submodule("platform");
  m_platform.attr("info") = as2_names::topics::platform::info;

  auto m_controller = m_topics.def_submodule("controller");
  m_controller.attr("info") = as2_names::topics::controller::info;

  auto m_follow_target = m_topics.def_submodule("follow_target");
  m_follow_target.attr("info") = as2_names::topics::follow_target::info;

  // SERVICES
  auto m_services = m.def_submodule("services", "Services name defaults");

  auto m_platform_serv = m_services.def_submodule("platform");
  m_platform_serv.attr("set_arming_state") = as2_names::services::platform::set_arming_state;
  m_platform_serv.attr("set_offboard_mode") = as2_names::services::platform::set_offboard_mode;
  m_platform_serv.attr("set_platform_control_mode") =
    as2_names::services::platform::set_platform_control_mode;
  m_platform_serv.attr("takeoff") = as2_names::services::platform::takeoff;
  m_platform_serv.attr("land") = as2_names::services::platform::land;
  m_platform_serv.attr("set_platform_state_machine_event") =
    as2_names::services::platform::set_platform_state_machine_event;
  m_platform_serv.attr("list_control_modes") = as2_names::services::platform::list_control_modes;

  auto m_controller_serv = m_services.def_submodule("controller");
  m_controller_serv.attr("set_control_mode") = as2_names::services::controller::set_control_mode;
  m_controller_serv.attr("list_control_modes") =
    as2_names::services::controller::list_control_modes;

  auto m_motion_reference_serv = m_services.def_submodule("motion_reference");
  m_motion_reference_serv.attr("send_traj_wayp") =
    as2_names::services::motion_reference::send_traj_wayp;
  m_motion_reference_serv.attr("add_traj_wayp") =
    as2_names::services::motion_reference::add_traj_wayp;
  m_motion_reference_serv.attr("set_traj_speed") =
    as2_names::services::motion_reference::set_traj_speed;

  auto m_gps = m_services.def_submodule("gps");
  m_gps.attr("get_origin") = as2_names::services::gps::get_origin;
  m_gps.attr("set_origin") = as2_names::services::gps::set_origin;
  m_gps.attr("path_to_geopath") = as2_names::services::gps::path_to_geopath;
  m_gps.attr("geopath_to_path") = as2_names::services::gps::geopath_to_path;

  auto m_behavior = m_services.def_submodule("behavior");
  m_behavior.attr("package_pickup") = as2_names::services::behavior::package_pickup;
  m_behavior.attr("package_unpick") = as2_names::services::behavior::package_unpick;
  m_behavior.attr("dynamic_land") = as2_names::services::behavior::dynamic_land;
  m_behavior.attr("dynamic_follower") = as2_names::services::behavior::dynamic_follower;

  m_services.attr("set_speed") = as2_names::services::set_speed;

  // ACTIONS
  auto m_actions = m.def_submodule("actions", "Actions name defaults");
  m_actions.attr("takeoff") = as2_names::actions::behaviors::takeoff;
  m_actions.attr("gotowaypoint") = as2_names::actions::behaviors::gotowaypoint;
  m_actions.attr("followreference") = as2_names::actions::behaviors::followreference;
  m_actions.attr("followpath") = as2_names::actions::behaviors::followpath;
  m_actions.attr("land") = as2_names::actions::behaviors::land;
  m_actions.attr("trajectorygenerator") = as2_names::actions::behaviors::trajectorygenerator;
}
