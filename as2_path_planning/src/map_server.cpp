#include "map_server.hpp"

MapServer::MapServer() : Node("map_server") {
  this->declare_parameter("map_resolution", 0.25); // [m/cell]
  map_resolution_ = this->get_parameter("map_resolution").as_double();

  this->declare_parameter("map_width", 300); // [cells]
  map_width_ = this->get_parameter("map_width").as_int();

  this->declare_parameter("map_height", 300); // [cells]
  map_height_ = this->get_parameter("map_height").as_int();

  occ_grid_sub_ =
      this->create_subscription<as2_msgs::msg::LabeledOccupancyGrid>(
          "input_occupancy_grid", 10,
          std::bind(&MapServer::occGridCallback, this, std::placeholders::_1));
  grid_map_pub_ =
      this->create_publisher<grid_map_msgs::msg::GridMap>("grid_map", 10);
  occ_grid_pub_ =
      this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);
  occ_grid_filter_pub_ =
      this->create_publisher<nav_msgs::msg::OccupancyGrid>("map_filtered", 10);

  show_map_serv_ = this->create_service<std_srvs::srv::Empty>(
      "save_map", std::bind(&MapServer::saveMapCallback, this,
                            std::placeholders::_1, std::placeholders::_2));

  // Empty map
  occ_grid_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  occ_grid_->header.stamp = this->now();
  occ_grid_->header.frame_id = "earth";
  occ_grid_->info.resolution = map_resolution_; // [m/cell]
  occ_grid_->info.width = map_width_;           // [cells]
  occ_grid_->info.height = map_height_;         // [cells]
  occ_grid_->info.origin.position.x =
      -occ_grid_->info.width / 2 * occ_grid_->info.resolution; // [m]
  occ_grid_->info.origin.position.y =
      -occ_grid_->info.height / 2 * occ_grid_->info.resolution; // [m]
  occ_grid_->data.assign(occ_grid_->info.width * occ_grid_->info.height,
                         -1); // unknown
};

void MapServer::occGridCallback(
    const as2_msgs::msg::LabeledOccupancyGrid::SharedPtr msg) {
  // Values at occ_grid are: 0 (free), 100 (occupied) or -1 (unknown)
  cv::Mat aux = cv::Mat(msg->occ_grid.data);
  aux.setTo(-10, aux == 0);  // free with weight -> 10
  aux.setTo(40, aux == 100); // occupied with weight -> 40
  aux.setTo(0, aux == -1);   // unknown with weight -> 0

  aux += cv::Mat(occ_grid_->data);
  aux.setTo(-1, aux == -1);
  aux.setTo(0, aux < -1);
  aux.setTo(100, aux > 100);

  occ_grid_->header.stamp = msg->occ_grid.header.stamp;
  occ_grid_->data = aux.clone();
  occ_grid_pub_->publish(*occ_grid_);

  // Filtering output map (Closing filter)
  cv::Mat map = utils::gridToImg(*(occ_grid_)).clone();
  cv::morphologyEx(map, map, cv::MORPH_CLOSE, cv::Mat());
  auto occ_grid_filtered = utils::imgToGrid(map, msg->occ_grid.header,
                                            msg->occ_grid.info.resolution);
  cv::Mat aux2 = cv::Mat(occ_grid_->data).clone();
  aux2.setTo(0, cv::Mat(occ_grid_filtered.data) == 0);
  occ_grid_filtered.header.stamp = msg->occ_grid.header.stamp;
  occ_grid_filtered.info = msg->occ_grid.info;
  occ_grid_filtered.data = aux2.clone();
  occ_grid_filter_pub_->publish(occ_grid_filtered);

  converter_.fromOccupancyGrid(occ_grid_filtered, msg->label, grid_map_);
  grid_map_msgs::msg::GridMap::UniquePtr grid_map_msg =
      converter_.toMessage(grid_map_);
  grid_map_pub_->publish(*grid_map_msg);
}

static void printMatInfo(const rclcpp::Node *node, cv::Mat mat) {
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

void MapServer::saveMapCallback(
    const std_srvs::srv::Empty::Request::SharedPtr request,
    std_srvs::srv::Empty::Response::SharedPtr response) {
  cv::Mat mat = utils::gridToImg(*(occ_grid_));

  cv::imwrite("/tmp/map.png", mat);
  RCLCPP_INFO(this->get_logger(), "Image saved at /tmp/map.png");
}

void MapServer::showMap(const cv::Mat &mat, std::string window_name) {
  cv::namedWindow(window_name, cv::WINDOW_NORMAL);
  cv::imshow(window_name, mat);
  cv::waitKey(0);
  cv::destroyAllWindows();
  cv::destroyWindow(window_name);
}
