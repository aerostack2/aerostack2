#include "map_server.hpp"

MapServer::MapServer() : Node("map_server") {
  occ_grid_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "input_occupancy_grid", 10,
      std::bind(&MapServer::occGridCallback, this, std::placeholders::_1));
  occ_grid_pub_ =
      this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);

  show_map_serv_ = this->create_service<std_srvs::srv::Empty>(
      "save_map", std::bind(&MapServer::saveMapCallback, this,
                            std::placeholders::_1, std::placeholders::_2));
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
