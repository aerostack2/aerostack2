// "Copyright [year] <Copyright Owner>"

#include <rclcpp/logging.hpp>
#include "as2_core/core_functions.hpp"
#include "dji_matrice_platform.hpp"

int main(int argc, char* argv[]) {
  std::cout << "Starting DJI Matrice Platform" << std::endl;
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("dji_matrice_platform"),
              "Starting DJI Matrice Platform");
  auto node = std::make_shared<DJIMatricePlatform>(argc, argv);
  node->preset_loop_frequency(300);
  node->start();
  // rclcpp::spin(node);
  as2::spinLoop(node);
  // node->run_test();

  rclcpp::shutdown();
  return 0;
}
