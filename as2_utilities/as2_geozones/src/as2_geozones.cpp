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
 *  \file       geozones.hpp
 *  \brief      Source file for geozones
 *  \authors    Javier Melero Deza
 *  \copyright  Copyright (c) 2024 Universidad Politécnica de Madrid
 *              All Rights Reserved
 ********************************************************************************/

#include "as2_geozones.hpp"

Geozones::Geozones()
: as2::Node("geozones")
{
  this->declare_parameter<std::string>(
    "config_file",
    "geozones/geofences.json");

  this->declare_parameter<bool>("debug_rviz", true);
}

void Geozones::run()
{
  point_ = {self_x_, self_y_};

  if (!start_run_) {
    return;
  }

  if (geozones_.size() == 0) {
    return;
  } else {
    checkGeozones();
  }
}

void Geozones::setupNode()
{
  // Set subscriptions, publishers and services.
  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    this->generate_global_name(as2_names::topics::self_localization::pose),
    as2_names::topics::self_localization::qos,
    std::bind(&Geozones::poseCallback, this, std::placeholders::_1));

  set_geozone_srv_ = this->create_service<as2_geozones::srv::SetGeozone>(
    this->generate_local_name("set_geozone"),
    std::bind(
      &Geozones::setGeozoneCb, this, std::placeholders::_1,
      std::placeholders::_2));

  get_geozone_srv_ = this->create_service<as2_geozones::srv::GetGeozone>(
    this->generate_local_name("get_geozone"),
    std::bind(
      &Geozones::getGeozoneCb, this, std::placeholders::_1,
      std::placeholders::_2));

  alert_pub_ = this->create_publisher<as2_msgs::msg::AlertEvent>(
    this->generate_global_name("alert_event"), 1);

  if (rviz_visualization_) {
    rviz_pub_ = this->create_publisher<as2_geozones::msg::Polygonlist>(
      this->generate_global_name("geozones_rviz"), 1);
    timer_ =
      this->create_timer(
      std::chrono::milliseconds(100),
      std::bind(&Geozones::rvizVisualizationCb, this));
  }

  loadGeozones(config_path_);
}

void Geozones::loadGeozones(const std::string path)
{
  // Load geozones from JSON file
  std::ifstream fJson(path);
  std::stringstream buffer;
  buffer << fJson.rdbuf();
  auto json = nlohmann::json::parse(buffer.str());

  for (auto json_geozone : json["geozones"]) {
    geozone geozone_to_load;
    if (!checkValidity(
        std::size(json_geozone["polygon"]), json_geozone["id"],
        json_geozone["type"], json_geozone["data_type"]))
    {
      RCLCPP_INFO(
        this->get_logger(),
        "Geozone %d not loaded from JSON file",
        json_geozone["id"]);
      continue;
    } else {
      geozone_to_load.data_type = json_geozone["data_type"];
      std::vector<std::array<double, 2>> polygon;
      if (geozone_to_load.data_type == "gps") {
        if (!origin_set_) {
          setupGPS();
          origin_set_ = true;
        }
        for (std::array<double, 2> point : json_geozone["polygon"]) {
          double z;
          gps_handler->LatLon2Local(
            point[0], point[1], 0.0, point[0], point[1],
            z);
          polygon.push_back(point);
        }
      } else {
        for (std::array<double, 2> point : json_geozone["polygon"]) {
          polygon.push_back(point);
        }
      }

      geozone_to_load.id = json_geozone["id"];
      geozone_to_load.alert = json_geozone["alert"];

      geozone_to_load.type = json_geozone["type"];
      geozone_to_load.data_type = json_geozone["data_type"];

      geozone_to_load.z_up = (json_geozone.contains("z_up")) ?
        static_cast<float>(json_geozone["z_up"]) :
        std::numeric_limits<float>::infinity();

      geozone_to_load.z_down = (json_geozone.contains("z_down")) ?
        static_cast<float>(json_geozone["z_down"]) :
        std::numeric_limits<float>::lowest();

      geozone_to_load.in = false;
      geozone_to_load.polygon = polygon;
      geozones_.push_back(geozone_to_load);

      RCLCPP_INFO(
        this->get_logger(),
        "Geostructure Succesfully loaded from JSON file");
    }
  }
}

void Geozones::checkGeozones()
{
  as2_msgs::msg::AlertEvent alert;
  for (std::vector<geozone>::iterator ptr = geozones_.begin();
    ptr < geozones_.end(); ptr++)
  {
    // auto [point, polygon] = translatePolygonWithPoint(ptr->polygon, point_);

    if (!Pnpoly::isIn<double>(ptr->polygon, point_) || self_z_ < ptr->z_down ||
      self_z_ > ptr->z_up)
    {
      if (ptr->type == "geocage") {
        alert.alert = ptr->alert;
        alert_pub_->publish(alert);
      }
      if (ptr->in == true) {
        ptr->in = false;
        RCLCPP_INFO(
          this->get_logger(), "Exited area: %s",
          std::to_string(ptr->id).c_str());
      }
    } else {
      if (ptr->type == "geofence") {
        alert.alert = ptr->alert;
        alert_pub_->publish(alert);
      }
      if (ptr->in == false) {
        ptr->in = true;
        RCLCPP_INFO(
          this->get_logger(), "Entered area: %s",
          std::to_string(ptr->id).c_str());
      }
    }
  }
}

bool Geozones::findGeozoneId(int id)
{
  for (std::vector<geozone>::iterator ptr = geozones_.begin();
    ptr < geozones_.end(); ptr++)
  {
    if (id == ptr->id) {
      return false;
    }
  }
  return true;
}

bool Geozones::checkValidity(
  int size, int id, std::string type,
  std::string data_type)
{
  if (!findGeozoneId(id)) {
    RCLCPP_WARN(this->get_logger(), "Id already exist.");
    return false;
  }

  if (type != "geofence" && type != "geocage") {
    RCLCPP_WARN(
      this->get_logger(),
      "Invalid type: %s. Allowed values: 'geofence', 'geocage'.",
      type.c_str());
    return false;
  }

  if (data_type != "gps" && data_type != "cartesian") {
    RCLCPP_WARN(
      this->get_logger(),
      "Invalid data type: %s. Allowed values: 'gps', 'cartesian'.",
      type.c_str());
    return false;
  }

  if (size < 3) {
    RCLCPP_WARN(
      this->get_logger(),
      "Invalid Geofence. Polygon must contain at least 3 points");
    return false;
  }

  return true;
}

void Geozones::setupGPS()
{
  get_origin_srv_ = this->create_client<as2_msgs::srv::GetOrigin>(
    as2_names::services::gps::get_origin);   // Should be same origin for every
                                             // drone ?
  while (!get_origin_srv_->wait_for_service()) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Interrupted while waiting for the service. Exiting.");
      break;
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }

  auto request = std::make_shared<as2_msgs::srv::GetOrigin::Request>();

  request->structure_needs_at_least_one_member = 0;

  bool success = false;  // TO-DO(javilinos): Improve this

  // Wait for the result.
  while (!success) {
    auto result = get_origin_srv_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(
        this->get_node_base_interface(),
        result, std::chrono::seconds(1)) ==
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(this->get_logger(), "Origin received");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to call service get origin");
      return;
    }
    auto result_obj = *result.get();
    success = result_obj.success;
    if (success) {
      origin_ =
        std::make_unique<geographic_msgs::msg::GeoPoint>(result_obj.origin);
      RCLCPP_INFO(
        this->get_logger(), "Origin in: lat: %f, lon %f, alt: %f",
        origin_->latitude, origin_->longitude, origin_->altitude);
      gps_handler = std::make_unique<as2::gps::GpsHandler>(
        origin_->latitude, origin_->longitude, origin_->altitude);
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "Get origin request not successful, trying again...");
    }
  }
}

// CALLBACKS //

void Geozones::poseCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr _msg)
{
  self_x_ = _msg->pose.position.x;
  self_y_ = _msg->pose.position.y;
  self_z_ = _msg->pose.position.z;

  start_run_ = true;
}

void Geozones::setGeozoneCb(
  const std::shared_ptr<as2_geozones::srv::SetGeozone::Request> request,
  std::shared_ptr<as2_geozones::srv::SetGeozone::Response> response)
{
  if (!checkValidity(
      std::size(request->geozone.polygon.points),
      request->geozone.id, request->geozone.type,
      request->geozone.data_type))
  {
    response->success = false;
  } else {
    geozone geozone_to_load;
    std::vector<std::array<double, 2>> polygon;
    for (int i = 0; i < std::size(request->geozone.polygon.points); i++) {
      std::array<double, 2> point{request->geozone.polygon.points[i].x,
        request->geozone.polygon.points[i].y};
      polygon.push_back(point);
    }
    geozone_to_load.id = request->geozone.id;
    geozone_to_load.alert = request->geozone.alert;
    geozone_to_load.type = request->geozone.type;
    geozone_to_load.data_type = request->geozone.data_type;
    geozone_to_load.z_up = request->geozone.z_up;
    geozone_to_load.z_down = request->geozone.z_down;
    geozone_to_load.in = false;
    geozone_to_load.polygon = polygon;
    geozones_.push_back(geozone_to_load);

    RCLCPP_INFO(this->get_logger(), "Geostructure added.");
    response->success = true;
  }
}

void Geozones::getGeozoneCb(
  const std::shared_ptr<as2_geozones::srv::GetGeozone::Request> request,
  std::shared_ptr<as2_geozones::srv::GetGeozone::Response> response)
{
  if (geozones_.size() == 0) {
    RCLCPP_WARN(this->get_logger(), "No geozone has been set yet.");
    response->success = false;
  } else {
    std::vector<as2_geozones::msg::Geozone> geozone_list;
    for (std::vector<geozone>::iterator ptr = geozones_.begin();
      ptr < geozones_.end(); ptr++)
    {
      as2_geozones::msg::Geozone geozone;
      for (std::vector<std::array<double, 2>>::iterator ptr2 =
        ptr->polygon.begin();
        ptr2 < ptr->polygon.end(); ptr2++)
      {
        geometry_msgs::msg::Point32 point;
        double x, y, z;
        if (ptr->data_type == "gps") {
          RCLCPP_INFO(
            this->get_logger(), "POINT: %f, %f", (*ptr2)[0],
            (*ptr2)[1]);
          gps_handler->Local2LatLon(
            (*ptr2)[0], (*ptr2)[1], 0.0, x, y,
            z);
          RCLCPP_INFO(this->get_logger(), "CONVERTED POINT: %f, %f", x, y);
          point.x = x;
          point.y = y;
          z = 0.0;
        } else {
          point.x = (*ptr2)[0];
          point.y = (*ptr2)[1];
        }

        geozone.polygon.points.push_back(point);
      }
      geozone.z_up = ptr->z_up;
      geozone.z_down = ptr->z_down;
      geozone.type = ptr->type;
      geozone.data_type = ptr->data_type;
      geozone.alert = ptr->alert;
      geozone.id = ptr->id;
      geozone_list.push_back(geozone);
    }
    response->geozone_list = geozone_list;
    response->success = true;
  }
}

void Geozones::rvizVisualizationCb()
{
  as2_geozones::msg::Polygonlist polygonlist;
  for (std::vector<geozone>::iterator geozone = geozones_.begin();
    geozone < geozones_.end(); geozone++)
  {
    geometry_msgs::msg::PolygonStamped polygon;
    polygon.header.frame_id = "earth";
    polygon.header.stamp = this->now();
    for (std::vector<std::array<double, 2>>::iterator poly_point =
      geozone->polygon.begin();
      poly_point < geozone->polygon.end(); poly_point++)
    {
      geometry_msgs::msg::Point32 point;
      point.x = (*poly_point)[0];
      point.y = (*poly_point)[1];
      polygon.polygon.points.push_back(point);
    }
    polygonlist.polygons.push_back(polygon);
  }
  rviz_pub_->publish(polygonlist);
}

void Geozones::cleanupNode()
{
  // TODO(javilinos): Cleanup Node
}

using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturn Geozones::on_configure(const rclcpp_lifecycle::State & _state)
{
  this->get_parameter("config_file", config_path_);
  this->get_parameter("debug_rviz", rviz_visualization_);

  setupNode();

  return CallbackReturn::SUCCESS;
}

CallbackReturn Geozones::on_activate(const rclcpp_lifecycle::State & _state)
{
  // Set parameters?

  return CallbackReturn::SUCCESS;
}

CallbackReturn Geozones::on_deactivate(const rclcpp_lifecycle::State & _state)
{
  // Clean up subscriptions, publishers, services, actions, etc. here.
  cleanupNode();

  return CallbackReturn::SUCCESS;
}

CallbackReturn Geozones::on_shutdown(const rclcpp_lifecycle::State & _state)
{
  // Clean other resources here.

  return CallbackReturn::SUCCESS;
}
