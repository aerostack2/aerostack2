#include <benchmark/benchmark.h>
#include <PID_3D.hpp>
#include <exception>

using namespace pid_controller;

using Vector3d = Eigen::Vector3d;
using Matrix3d = Eigen::Matrix3d;

static void BM_TEST(benchmark::State &state) {
  PIDController3D controller = PIDController3D();
  Vector3d input             = Vector3d::Ones();
  bool flag                  = true;

  controller.setGains(input, input, input);
  controller.setAntiWindup(input);
  controller.setAlpha(input);
  controller.setResetIntegralSaturationFlag(flag);

  double dt = 0.01;

  Vector3d state_vec = Vector3d::Ones();
  Vector3d ref_vec   = 2.0 * Vector3d::Ones();
  for (auto _ : state) {
    controller.computeControl(dt, state_vec, ref_vec);
  }
}
BENCHMARK(BM_TEST)->Threads(1)->Repetitions(10);

int main(int argc, char **argv) {
  // benchmark::RegisterBenchmark("run Efficiciency", BM_TEST);
  benchmark::Initialize(&argc, argv);
  benchmark::RunSpecifiedBenchmarks();
  benchmark::Shutdown();
}