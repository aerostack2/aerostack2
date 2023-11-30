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
  appendGridMap(msg);
  // TODO: change global map to be obtained from layers in grid_map

  occ_grid_->header.stamp = msg->occ_grid.header.stamp;
  occ_grid_->data =
      cv::Mat(addOccGridUpdate(msg->occ_grid.data, occ_grid_->data));
  occ_grid_pub_->publish(*occ_grid_);

  nav_msgs::msg::OccupancyGrid occ_grid_filtered = filterOccGrid(*occ_grid_);
  occ_grid_filter_pub_->publish(occ_grid_filtered);
}

void MapServer::appendGridMap(
    const as2_msgs::msg::LabeledOccupancyGrid::SharedPtr msg) {
  nav_msgs::msg::OccupancyGrid target_occ_grid;
  if (grid_map_.exists(msg->label)) {
    converter_.toOccupancyGrid(grid_map_, msg->label, -1, 100, target_occ_grid);
  } else {
    target_occ_grid = msg->occ_grid;
    RCLCPP_INFO(this->get_logger(), "Creating new layer: %s",
                msg->label.c_str());
  }

  target_occ_grid.data =
      cv::Mat(addOccGridUpdate(msg->occ_grid.data, target_occ_grid.data));
  nav_msgs::msg::OccupancyGrid occ_grid_filtered =
      filterOccGrid(target_occ_grid);

  converter_.fromOccupancyGrid(occ_grid_filtered, msg->label, grid_map_);
  grid_map_msgs::msg::GridMap::UniquePtr grid_map_msg =
      converter_.toMessage(grid_map_);
  grid_map_pub_->publish(*grid_map_msg);
}

std::vector<int8_t>
MapServer::addOccGridUpdate(const std::vector<int8_t> &update,
                            const std::vector<int8_t> &occ_grid_data) {
  // Values at occ_grid update are: 0 (free), 100 (occupied) or -1 (unknown)
  cv::Mat aux = cv::Mat(update).clone();
  aux.setTo(-10, aux == 0);  // free with weight -> 10
  aux.setTo(40, aux == 100); // occupied with weight -> 40
  aux.setTo(0, aux == -1);   // unknown with weight -> 0

  aux += cv::Mat(occ_grid_data);
  aux.setTo(-1, aux == -1);
  aux.setTo(0, aux < -1);
  aux.setTo(100, aux > 100);

  // Keeping obstacles
  aux.setTo(100, cv::Mat(occ_grid_data) > 80);
  return aux;
}

nav_msgs::msg::OccupancyGrid
MapServer::filterOccGrid(const nav_msgs::msg::OccupancyGrid &occ_grid) {
  // Filtering output map (Closing filter)
  cv::Mat map = utils::gridToImg(occ_grid).clone(); // copy of grid
  cv::morphologyEx(map, map, cv::MORPH_CLOSE, cv::Mat());
  nav_msgs::msg::OccupancyGrid occ_grid_filtered =
      utils::imgToGrid(map, occ_grid.header, occ_grid.info.resolution);
  cv::Mat aux2 = cv::Mat(occ_grid.data).clone();
  aux2.setTo(0, cv::Mat(occ_grid_filtered.data) == 0);
  aux2.setTo(100, cv::Mat(occ_grid.data) == 100); // obstacles not filtered

  occ_grid_filtered.data = aux2.clone();
  return occ_grid_filtered;
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
