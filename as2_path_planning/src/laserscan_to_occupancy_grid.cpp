#include "laserscan_to_occupancy_grid.hpp"

LaserToOccupancyGridNode::LaserToOccupancyGridNode()
    : Node("laser_to_occupancy_grid_node") {
  this->declare_parameter("map_resolution", 0.25); // [m/cell]
  map_resolution_ = this->get_parameter("map_resolution").as_double();

  this->declare_parameter("map_width", 300); // [cells]
  map_width_ = this->get_parameter("map_width").as_int();

  this->declare_parameter("map_height", 300); // [cells]
  map_height_ = this->get_parameter("map_height").as_int();

  this->declare_parameter("max_range_limit", 10.0); // [m]
  max_range_limit_ = (float)this->get_parameter("max_range_limit").as_double();

  laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "sensor_measurements/lidar/scan",
      as2_names::topics::sensor_measurements::qos,
      std::bind(&LaserToOccupancyGridNode::processLaserScan, this,
                std::placeholders::_1));

  occupancy_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "debug/occ_grid", 10);

  labeled_occupancy_grid_pub_ =
      this->create_publisher<as2_msgs::msg::LabeledOccupancyGrid>(
          "labeled_occ_grid", 10);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  RCLCPP_INFO(this->get_logger(), "Node up and running");
}

void LaserToOccupancyGridNode::processLaserScan(
    const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  nav_msgs::msg::OccupancyGrid occupancy_grid_msg;
  occupancy_grid_msg.header.stamp = scan->header.stamp;
  occupancy_grid_msg.header.frame_id = "earth";
  occupancy_grid_msg.info.width = map_width_;           // [cell]
  occupancy_grid_msg.info.height = map_height_;         // [cell]
  occupancy_grid_msg.info.resolution = map_resolution_; // [m/cell]
  // Origen del sistema de coordenadas
  occupancy_grid_msg.info.origin.position.x =
      -map_width_ / 2 * map_resolution_; // [m]
  occupancy_grid_msg.info.origin.position.y =
      -map_height_ / 2 * map_resolution_; // [m]

  // Inicializar todos los valores con un valor neutral (desconocido)
  occupancy_grid_msg.data.assign(
      occupancy_grid_msg.info.width * occupancy_grid_msg.info.height, -1);

  // Free cells
  geometry_msgs::msg::PointStamped drone_pose;
  drone_pose.header = scan->header;
  drone_pose.point.x = 0;
  drone_pose.point.y = 0;
  std::vector<int> drone_cell;
  try {
    drone_cell =
        utils::pointToCell(drone_pose, occupancy_grid_msg.info,
                           occupancy_grid_msg.header.frame_id, tf_buffer_);
    /* Moved to MapServer after eroding obstacles, otherwise it will be done
     * more than once
     *
      std::vector<std::vector<int>> free_cells;
      free_cells = safeZone(drone_cell);
     *
     */
  } catch (const tf2::ExtrapolationException &e) {
    std::cerr << e.what() << '\n';
    return;
  }

  // Calcular las coordenadas de celda para cada punto del escaneo láser
  for (size_t i = 0; i < scan->ranges.size(); ++i) {

    float angle = scan->angle_min + i * scan->angle_increment;

    // Si inf el láser no ha detectado ningún obstáculo, se pone el rango max
    if (std::isinf(scan->ranges[i])) {
      scan->ranges[i] = scan->range_max;
    }

    if (scan->ranges[i] < scan->range_min) {
      continue;
    }

    // Limitar el rango máximo del láser por posible ruido a valores extremos
    if (scan->ranges[i] > max_range_limit_) {
      scan->ranges[i] = max_range_limit_;
    }

    geometry_msgs::msg::PointStamped in;
    in.header = scan->header;
    in.point.x = scan->ranges[i] * std::cos(angle); // Convertir a coordenada x
    in.point.y = scan->ranges[i] * std::sin(angle); // Convertir a coordenada y
    std::vector<int> cell;
    try {
      cell = utils::pointToCell(in, occupancy_grid_msg.info,
                                occupancy_grid_msg.header.frame_id, tf_buffer_);
    } catch (const tf2::ExtrapolationException &e) {
      std::cerr << e.what() << '\n';
      continue;
    }

    // Los puntos intermedios entre el robot y el punto de escaneo láser
    // están siempre libres, se detecte un obstaculo o no
    std::vector<std::vector<int>> middle_cells =
        getMiddlePoints(drone_cell, cell);

    /* Middle free points filled in grid just after getting them, otherwise
     * they will override obstacles. Check if getMiddlePoints is adding extra
     * free points
     *
      free_cells.insert(free_cells.end(), middle_cells.begin(),
                        middle_cells.end());
     *
     */
    for (const std::vector<int> &p : middle_cells) {
      int cell_index = p[1] * occupancy_grid_msg.info.width + p[0];
      if (isCellIndexValid(p)) {
        occupancy_grid_msg.data[cell_index] = 0; // free
      }
    }

    // Actualizar la ocupación de la celda según el umbral de ocupación
    int cell_index = cell[1] * occupancy_grid_msg.info.width + cell[0];
    if (isCellIndexValid(cell)) {
      // Umbral de ocupación
      occupancy_grid_msg.data[cell_index] =
          (scan->ranges[i] < std::min(scan->range_max, max_range_limit_)) ? 100
                                                                          : 0;
    }
  }

  occupancy_grid_pub_->publish(occupancy_grid_msg);
  as2_msgs::msg::LabeledOccupancyGrid labeled_occ_grid_msg;
  labeled_occ_grid_msg.label = this->get_namespace();
  labeled_occ_grid_msg.occ_grid = occupancy_grid_msg;
  labeled_occupancy_grid_pub_->publish(labeled_occ_grid_msg);
}

std::vector<std::vector<int>>
LaserToOccupancyGridNode::getMiddlePoints(std::vector<int> p1,
                                          std::vector<int> p2) {
  std::vector<std::vector<int>> middle_points;
  int dx = p2[0] - p1[0];
  int dy = p2[1] - p1[1];
  int steps = std::max(std::abs(dx), std::abs(dy));
  float xinc = dx / (float)steps;
  float yinc = dy / (float)steps;

  float x = p1[0];
  float y = p1[1];

  for (int i = 0; i < steps - 1; ++i) {
    x += xinc;
    y += yinc;
    middle_points.push_back({(int)std::round(x), (int)std::round(y)});
  }

  return middle_points;
};

bool LaserToOccupancyGridNode::isCellIndexValid(std::vector<int> cell) {
  return cell[0] >= 0 && cell[0] < map_width_ && cell[1] >= 0 &&
         cell[1] < map_height_;
}
