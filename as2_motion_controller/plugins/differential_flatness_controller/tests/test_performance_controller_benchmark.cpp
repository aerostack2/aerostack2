#include <benchmark/benchmark.h>

#include <exception>
#include "rclcpp/rclcpp.hpp"
// #include "DF_controller.hpp"
/*
PD_controller *ptr = nullptr;

static void BM_COMPUTE_ACTIONS(benchmark::State &state)
{
  // ptr->setup();
  for (auto _ : state)
  {
    ptr->computeActions();
  }
}
BENCHMARK(BM_COMPUTE_ACTIONS)->Threads(1)->Repetitions(10);

static void BM_ODOM_CB(benchmark::State &state)
{
  ptr->setup();
  for (auto _ : state)
  {
    ptr->CallbackOdomTopic(std::make_shared<nav_msgs::msg::Odometry>());
  }
}
BENCHMARK(BM_ODOM_CB)->Threads(1)->Repetitions(10);

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  ptr = new PD_controller();
  if (
      rcutils_logging_set_logger_level(ptr->get_logger().get_name(),
RCUTILS_LOG_SEVERITY_WARN) == RCUTILS_RET_ERROR) throw std::runtime_error("Error
setting logger level");

  // benchmark::RegisterBenchmark("run Efficiciency", BM_COMPUTE_ACTIONS);
  benchmark::Initialize(&argc, argv);
  benchmark::RunSpecifiedBenchmarks();
  benchmark::Shutdown();
  delete ptr;
}
*/
