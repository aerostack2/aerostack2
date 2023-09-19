#include "map_server.hpp"

MapServer::MapServer() : Node("map_server") {
  occ_grid_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "output_occupancy_grid", 10,
      std::bind(&MapServer::occGridCallback, this, std::placeholders::_1));

  drone_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "self_localization/pose", as2_names::topics::self_localization::qos,
      std::bind(&MapServer::dronePoseCbk, this, std::placeholders::_1));

  debug_point_sub_ =
      this->create_subscription<geometry_msgs::msg::PointStamped>(
          "/clicked_point", 10,
          std::bind(&MapServer::clickedPointCallback, this,
                    std::placeholders::_1));

  occ_grid_pub_ =
      this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);
  obstacle_grid_pub_ =
      this->create_publisher<nav_msgs::msg::OccupancyGrid>("obstacle_map", 10);
  viz_pub_ =
      this->create_publisher<visualization_msgs::msg::Marker>("marker", 10);

  show_map_serv_ = this->create_service<std_srvs::srv::Empty>(
      "show_map", std::bind(&MapServer::showMapCallback, this,
                            std::placeholders::_1, std::placeholders::_2));

  follow_path_client_ =
      rclcpp_action::create_client<as2_msgs::action::FollowPath>(
          this, as2_names::actions::behaviors::followpath);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
};

void MapServer::occGridCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr occ_grid) {
  if (last_occ_grid_ == nullptr) {
    RCLCPP_INFO(this->get_logger(), "First map received");
    last_occ_grid_ = occ_grid;
    return;
  }
  // Todos los valores en occ_grid son 0 (libre), 100 (ocupado) o -1
  // (desconocido)
  cv::Mat aux = cv::Mat(occ_grid->data);
  aux.setTo(-10, aux == 0);  // libres con peso 10
  aux.setTo(40, aux == 100); // ocupados con peso 40
  aux.setTo(0, aux == -1);   // desconocidos con valor 0

  aux += cv::Mat(last_occ_grid_->data);
  aux.setTo(-1, aux == -1);
  aux.setTo(0, aux < -1);
  aux.setTo(100, aux > 100);

  last_occ_grid_->data = aux.clone();
  occ_grid_pub_->publish(*last_occ_grid_);
}

static void printMatInfo(const rclcpp::Node *node, cv::Mat mat) {
  // print mat2 type and number of channels
  RCLCPP_INFO(node->get_logger(), "Mat type: %d", mat.type());
  if (mat.type() == CV_8U) {
    RCLCPP_INFO(node->get_logger(), "Mat type is CV_8U");
  } else if (mat.type() == CV_8S) {
    RCLCPP_INFO(node->get_logger(), "Mat type is CV_8S");
  }

  RCLCPP_INFO(node->get_logger(), "Mat channels: %d", mat.channels());
  if (mat.channels() == 1) {
    double minVal, maxVal;
    cv::Point minLoc, maxLoc;
    cv::minMaxLoc(mat, &minVal, &maxVal, &minLoc, &maxLoc);
    RCLCPP_INFO(node->get_logger(), "Map max value %f at (%d, %d)", maxVal,
                maxLoc.x, maxLoc.y);
    RCLCPP_INFO(node->get_logger(), "Map min value %f at (%d, %d)", minVal,
                minLoc.x, minLoc.y);
  }
}

static visualization_msgs::msg::Marker
getPointMarker(std::string frame_id, rclcpp::Time stamp, cv::Point2i pixel,
               nav_msgs::msg::MapMetaData map_info,
               std_msgs::msg::Header map_header) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.ns = "map_server";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  marker.scale.x = 0.25; // Set the scale of the marker
  marker.scale.y = 0.25; // Set the scale of the marker
  marker.scale.z = 0.25; // Set the scale of the marker
  marker.lifetime =
      rclcpp::Duration::from_seconds(0); // Set the lifetime of the marker

  auto point = utils::pixelToPoint(pixel, map_info, map_header);

  marker.pose.position = point.point;
  return marker;
}

static visualization_msgs::msg::Marker
getPointMarker(std::string frame_id, rclcpp::Time stamp,
               geometry_msgs::msg::Point point) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.ns = "map_server";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  marker.scale.x = 0.25; // Set the scale of the marker
  marker.scale.y = 0.25; // Set the scale of the marker
  marker.scale.z = 0.25; // Set the scale of the marker
  marker.lifetime =
      rclcpp::Duration::from_seconds(0); // Set the lifetime of the marker

  marker.pose.position = point;
  return marker;
}

static visualization_msgs::msg::Marker
getPathMarker(std::string frame_id, rclcpp::Time stamp,
              std::vector<cv::Point> path, nav_msgs::msg::MapMetaData map_info,
              std_msgs::msg::Header map_header) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.ns = "map_server";
  marker.id = 33;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.1;
  marker.lifetime = rclcpp::Duration::from_seconds(0); // Lifetime forever

  for (auto &p : path) {
    auto point = utils::pixelToPoint(p, map_info, map_header);
    marker.points.push_back(point.point);
    std_msgs::msg::ColorRGBA color;
    color.a = 1.0;
    color.r = 0.0;
    color.g = 0.0;
    color.b = 1.0;
    marker.colors.push_back(color);
  }
  return marker;
}

void MapServer::clickedPointCallback(
    const geometry_msgs::msg::PointStamped point) {
  // Deep copy to avoid modifications on the original map
  nav_msgs::msg::OccupancyGrid::SharedPtr occ_grid =
      std::make_shared<nav_msgs::msg::OccupancyGrid>();
  occ_grid->header = last_occ_grid_->header;
  occ_grid->info = last_occ_grid_->info;
  occ_grid->data = last_occ_grid_->data;

  // World to image transformations
  cv::Point2i goal = utils::pointToPixel(
      point, last_occ_grid_->info, last_occ_grid_->header.frame_id, tf_buffer_);

  geometry_msgs::msg::PointStamped drone_pose;
  drone_pose.header = drone_pose_.header;
  drone_pose.point = drone_pose_.pose.position;
  cv::Point2i origin =
      utils::pointToPixel(drone_pose, last_occ_grid_->info,
                          last_occ_grid_->header.frame_id, tf_buffer_);

  // Erode obstacles
  cv::Mat mat = utils::gridToImg(occ_grid);
  cv::erode(mat, mat, cv::Mat());
  auto safe_cells = safeZone(origin, last_occ_grid_->info.resolution);
  for (const cv::Point2i &p : safe_cells) {
    mat.at<uchar>(p) = 255; // free
  }

  planner_algorithm_.setOriginPoint(origin);
  planner_algorithm_.setGoal(goal);
  planner_algorithm_.setOcuppancyGrid(mat);
  auto current_path_ = planner_algorithm_.solveGraph();
  RCLCPP_INFO(this->get_logger(), "Path size: %ld", current_path_.size());

  // Visualize path
  auto path_marker = getPathMarker(
      last_occ_grid_->header.frame_id, this->get_clock()->now(), current_path_,
      last_occ_grid_->info, last_occ_grid_->header);
  viz_pub_->publish(path_marker);

  // TEST
  mat.at<uchar>(origin.x - 1, origin.y - 1) = 128;
  mat.at<uchar>(goal.x - 1, goal.y - 1) = 128;
  auto test =
      utils::imgToGrid(mat, occ_grid->header, occ_grid->info.resolution);
  obstacle_grid_pub_->publish(test);
  // END TEST

  // Follow Path behavior
  callFollowPathAction(path_marker.points);
}

void MapServer::callFollowPathAction(
    std::vector<geometry_msgs::msg::Point> points) {
  if (!this->follow_path_client_->wait_for_action_server(
          std::chrono::seconds(5))) {
    RCLCPP_ERROR(this->get_logger(),
                 "Action server not available after waiting");
    return;
  }

  auto goal_msg = as2_msgs::action::FollowPath::Goal();
  goal_msg.header.frame_id = "earth";
  goal_msg.header.stamp = this->get_clock()->now();
  goal_msg.yaw.mode = as2_msgs::msg::YawMode::PATH_FACING;
  goal_msg.max_speed = 1.0;
  int i = 0;
  for (auto &p : points) {
    as2_msgs::msg::PoseWithID pid = as2_msgs::msg::PoseWithID();
    pid.id = std::to_string(i);
    pid.pose.position = p;
    pid.pose.position.z = 1.0;
    goal_msg.path.push_back(pid);
    i++;
  }

  RCLCPP_INFO(this->get_logger(), "Sending goal");

  follow_path_client_->async_send_goal(goal_msg);
}

void MapServer::showMapCallback(
    const std_srvs::srv::Empty::Request::SharedPtr request,
    std_srvs::srv::Empty::Response::SharedPtr response) {
  // cv::Mat mat =
  //     cv::Mat(last_occ_grid_->data).reshape(1, last_occ_grid_->info.width);
  // cv::Mat mat2;
  // cv::resize(mat, mat2, cv::Size(), 2, 2, cv::INTER_NEAREST);
  // mat2.setTo(255, mat2 == 0);
  // mat2.setTo(0, mat2 == 100);
  // cv::transpose(mat2, mat2);
  // cv::flip(mat2, mat2, 0);

  // create a Mat full of 255

  cv::Mat map = cv::Mat::ones(500, 500, CV_8UC1) * 255;
  for (int i = 0; i < 500; i++) {
    for (int j = 0; j < 500; j++) {
      if (i < 50 || i > 450 || j < 50 || j > 450) {
        map.at<uchar>(i, j) = 0;
      }

      if (i > 250 && i < 300 && j > 250 && j < 300) {
        map.at<uchar>(i, j) = 0;
      }
    }
  }

  cv::Point2i drone_origin = cv::Point2i(275, 400);
  cv::Point2i drone_goal = cv::Point2i(275, 100);
  planner_algorithm_.setOriginPoint(drone_origin);
  planner_algorithm_.setGoal(drone_goal);
  planner_algorithm_.setOcuppancyGrid(map);
  auto current_path_ = planner_algorithm_.solveGraph();
  RCLCPP_INFO(this->get_logger(), "Path size: %ld", current_path_.size());

  for (auto &p : current_path_) {
    map.at<uchar>(p.y, p.x) = 200;
  }

  RCLCPP_INFO(this->get_logger(), "Mat2 type: %d", map.type());
  RCLCPP_INFO(this->get_logger(), "Mat2 channels: %d", map.channels());
  double minVal;
  double maxVal;
  cv::Point minLoc;
  cv::Point maxLoc;
  // cv::minMaxIdx(map, &minVal, &maxVal);
  cv::minMaxLoc(map, &minVal, &maxVal, &minLoc, &maxLoc);
  RCLCPP_INFO(this->get_logger(), "Map max value %f at (%d, %d)", maxVal,
              maxLoc.x, maxLoc.y);
  RCLCPP_INFO(this->get_logger(), "Map min value %f at (%d, %d)", minVal,
              minLoc.x, minLoc.y);

  showMap(map);
  RCLCPP_INFO(this->get_logger(), "Map showed");
}

void MapServer::showMap(const cv::Mat &mat, std::string window_name) {
  cv::namedWindow(window_name, cv::WINDOW_NORMAL);
  cv::imshow(window_name, mat);
  cv::waitKey(0);
  cv::destroyAllWindows();
  cv::destroyWindow(window_name);
}

void MapServer::dronePoseCbk(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  drone_pose_ = *(msg);
}

struct Point2iHash {
  std::size_t operator()(const cv::Point2i &point) const {
    // Combine the hash values of x and y using a simple hash function
    return std::hash<int>()(point.x) ^ std::hash<int>()(point.y);
  }
};

std::vector<cv::Point2i> MapServer::safeZone(cv::Point2i point,
                                             double grid_resolution) {
  float security_distance = 0.5;
  int security_cells = std::round(security_distance / grid_resolution);

  std::unordered_set<cv::Point2i, Point2iHash> safe_zone;
  for (int i = 0; i < security_cells; ++i) {
    for (int j = 0; j < security_cells; ++j) {
      safe_zone.insert({point.y + i - 1, point.x + j - 1});
      safe_zone.insert({point.y - i - 1, point.x - j - 1});
      safe_zone.insert({point.y + i - 1, point.x - j - 1});
      safe_zone.insert({point.y - i - 1, point.x + j - 1});
    }
  }
  std::vector<cv::Point2i> pointVector(safe_zone.begin(), safe_zone.end());
  return pointVector;
}