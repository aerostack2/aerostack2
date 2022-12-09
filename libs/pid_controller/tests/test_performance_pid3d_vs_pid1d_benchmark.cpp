#include <benchmark/benchmark.h>
#include <PID.hpp>
#include <PID_3D.hpp>
#include <exception>

using namespace pid_controller;

using Vector3d = Eigen::Vector3d;
using Matrix3d = Eigen::Matrix3d;

static void BM_PID_3D(benchmark::State &state) {
  PIDController3D controller = PIDController3D();
  Vector3d input             = Vector3d::Ones();
  bool flag                  = false;

  controller.setGains(input, input, input);
  controller.setAntiWindup(input);
  controller.setAlpha(input);
  controller.setResetIntegralSaturationFlag(flag);

  double dt = 0.01;

  Vector3d state_vec = Vector3d::Ones();
  Vector3d ref_vec   = 2.0 * Vector3d::Ones();

  controller.computeControl(dt, state_vec, ref_vec);
  for (auto _ : state) {
    controller.computeControl(dt, state_vec, ref_vec);
  }
}
BENCHMARK(BM_PID_3D)->Threads(1)->Repetitions(10);

static void BM_PID_1D(benchmark::State &state) {
  PIDController controller = PIDController();
  double input             = 1.0;
  bool flag                = false;

  controller.setGains(input, input, input);
  controller.setAntiWindup(input);
  controller.setAlpha(input);
  controller.setResetIntegralSaturationFlag(flag);

  double dt = 0.01;

  double state_d = 1.0;
  double ref_d   = 2.0 * state_d;

  controller.computeControl(dt, state_d, ref_d);

  for (auto _ : state) {
    controller.computeControl(dt, state_d, ref_d);
  }
}
BENCHMARK(BM_PID_1D)->Threads(1)->Repetitions(10);

static void BM_PID_MIX(benchmark::State &state) {
  PIDController controller = PIDController();
  Vector3d state_v         = Vector3d::Ones();
  Vector3d reference_v     = 2 * Vector3d::Ones();

  double value = 1.0;
  bool flag    = false;

  controller.setGains(value, value, value);
  controller.setAntiWindup(value);
  controller.setAlpha(value);
  controller.setResetIntegralSaturationFlag(flag);

  double dt = 0.01;
  controller.computeControl(dt, state_v.x(), reference_v.x());
  controller.computeControl(dt, state_v.y(), reference_v.y());
  controller.computeControl(dt, state_v.z(), reference_v.z());

  for (auto _ : state) {
    controller.computeControl(dt, state_v.x(), reference_v.x());
    controller.computeControl(dt, state_v.y(), reference_v.y());
    controller.computeControl(dt, state_v.z(), reference_v.z());
  }
}
BENCHMARK(BM_PID_MIX)->Threads(1)->Repetitions(10);

int main(int argc, char **argv) {
  // benchmark::RegisterBenchmark("run Efficiciency", BM_TEST);
  benchmark::Initialize(&argc, argv);
  benchmark::RunSpecifiedBenchmarks();
  benchmark::Shutdown();
}