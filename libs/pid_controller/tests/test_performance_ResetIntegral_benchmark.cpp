#include <benchmark/benchmark.h>
#include <exception>

#include <math.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>

using Vector3d = Eigen::Vector3d;
using Matrix3d = Eigen::Matrix3d;

bool reset_integral_flag_     = true;
Vector3d integral_accum_error = 5 * Vector3d::Ones();
Vector3d antiwindup_cte_      = Vector3d::Ones();
double antiwindup_cte_d       = 1.0;
Vector3d _proportional_error  = -1.0 * Vector3d::Ones();

Vector3d resetIntegral1(Vector3d &integral_accum_error_, const bool &proportional_limitation) {
  if (reset_integral_flag_) {
    for (short j = 0; j < 3; j++) {
      if (std::abs(integral_accum_error_[j]) > antiwindup_cte_[j]) {
        if (std::signbit(integral_accum_error_[j]) != std::signbit(_proportional_error[j])) {
          integral_accum_error_[j] = 0.0f;
        }
      }
    }
  }
  return integral_accum_error_;
}

Vector3d resetIntegral2(Vector3d &integral_accum_error_, const bool &proportional_limitation) {
  if (reset_integral_flag_) {
    for (short j = 0; j < 3; j++) {
      if (std::abs(integral_accum_error_[j]) > antiwindup_cte_[j]) {
        if (std::signbit(integral_accum_error_[j]) != std::signbit(_proportional_error[j])) {
          integral_accum_error_[j] = 0.0f;
        }
      }
    }
  }
  return integral_accum_error_;
}

static void BM_TEST_RESETINTEGRAL1(benchmark::State &state) {
  bool reset_integral_flag_     = true;
  Vector3d integral_accum_error = 5 * Vector3d::Ones();
  Vector3d antiwindup_cte_      = Vector3d::Ones();
  double antiwindup_cte_d       = 1.0;
  Vector3d _proportional_error  = -1.0 * Vector3d::Ones();
  for (auto _ : state) {
    Vector3d output = resetIntegral1(integral_accum_error, reset_integral_flag_);
  }
  //   std::cout << "output: " << integral_accum_error << std::endl;
}
BENCHMARK(BM_TEST_RESETINTEGRAL1)->Threads(1)->Repetitions(20);

static void BM_TEST_RESETINTEGRAL2(benchmark::State &state) {
  bool reset_integral_flag_     = true;
  Vector3d integral_accum_error = 5 * Vector3d::Ones();
  Vector3d antiwindup_cte_      = Vector3d::Ones();
  double antiwindup_cte_d       = 1.0;
  Vector3d _proportional_error  = -1.0 * Vector3d::Ones();
  for (auto _ : state) {
    Vector3d output = resetIntegral2(integral_accum_error, reset_integral_flag_);
  }
  //   std::cout << "output: " << integral_accum_error << std::endl;
}
BENCHMARK(BM_TEST_RESETINTEGRAL2)->Threads(1)->Repetitions(20);

int main(int argc, char **argv) {
  // benchmark::RegisterBenchmark("run Efficiciency", BM_TEST);
  benchmark::Initialize(&argc, argv);
  benchmark::RunSpecifiedBenchmarks();
  benchmark::Shutdown();
}