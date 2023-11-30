#include "map_server.hpp"

MapServer::MapServer() : Node("map_server") {
  this->declare_parameter("map_resolution", 0.25); // [m/cell]
  map_resolution_ = this->get_parameter("map_resolution").as_double();

  this->declare_parameter("map_width", 300); // [cells]
  map_width_ = this->get_parameter("map_width").as_int();

  this->declare_parameter("map_height", 300); // [cells]
  map_height_ = this->get_parameter("map_height").as_int();

  occ_grid_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "input_occupancy_grid", 10,
      std::bind(&MapServer::occGridCallback, this, std::placeholders::_1));
  occ_grid_pub_ =
      this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);
  occ_grid_filter_pub_ =
      this->create_publisher<nav_msgs::msg::OccupancyGrid>("map_filtered", 10);

  show_map_serv_ = this->create_service<std_srvs::srv::Empty>(
      "save_map", std::bind(&MapServer::saveMapCallback, this,
                            std::placeholders::_1, std::placeholders::_2));

  // Empty map
  last_occ_grid_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  last_occ_grid_->header.stamp = this->now();
  last_occ_grid_->header.frame_id = "earth";
  last_occ_grid_->info.resolution = map_resolution_; // [m/cell]
  last_occ_grid_->info.width = map_width_;           // [cells]
  last_occ_grid_->info.height = map_height_;         // [cells]
  last_occ_grid_->info.origin.position.x =
      -last_occ_grid_->info.width / 2 * last_occ_grid_->info.resolution; // [m]
  last_occ_grid_->info.origin.position.y =
      -last_occ_grid_->info.height / 2 * last_occ_grid_->info.resolution; // [m]
  last_occ_grid_->data.assign(
      last_occ_grid_->info.width * last_occ_grid_->info.height, -1); // unknown
};

void MapServer::occGridCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr occ_grid) {
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

  last_occ_grid_->header = occ_grid->header;
  last_occ_grid_->data = aux.clone();
  occ_grid_pub_->publish(*last_occ_grid_);

  // Filtering output map (Closing filter)
  cv::Mat map = utils::gridToImg(*(last_occ_grid_)).clone();
  cv::morphologyEx(map, map, cv::MORPH_CLOSE, cv::Mat());
  auto occ_grid_filtered =
      utils::imgToGrid(map, occ_grid->header, occ_grid->info.resolution);
  cv::Mat aux2 = cv::Mat(last_occ_grid_->data).clone();
  aux2.setTo(0, cv::Mat(occ_grid_filtered.data) == 0);
  occ_grid_filtered.header = occ_grid->header;
  occ_grid_filtered.info = occ_grid->info;
  occ_grid_filtered.data = aux2.clone();
  occ_grid_filter_pub_->publish(occ_grid_filtered);
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
  cv::Mat mat = utils::gridToImg(*(last_occ_grid_));

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
