#include <benchmark/benchmark.h>
#include <exception>

#include <math.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>

using Vector3d = Eigen::Vector3d;
using Matrix3d = Eigen::Matrix3d;

Vector3d limitOutput1(const Vector3d &output,
                      const Vector3d &limits,
                      const bool &proportional_limitation) {
  Vector3d limited_output = output;

  // Delimit the speed for each axis
  if (proportional_limitation) {
    for (short j = 0; j < 3; j++) {
      if (limits[j] == 0.0f || output[j] == 0.0) {
        continue;
      };

      if (limited_output[j] > limits[j] || limited_output[j] < -limits[j]) {
        limited_output *= std::abs(limits[j] / limited_output[j]);
      }
    }
  } else {
    for (short j = 0; j < 3; j++) {
      if (limits[j] == 0.0f) {
        continue;
      }
      limited_output[j] = (limited_output[j] > limits[j]) ? limits[j] : limited_output[j];
      limited_output[j] = (limited_output[j] < -limits[j]) ? -limits[j] : limited_output[j];
    }
  }
  return limited_output;
}

Vector3d limitOutput2(const Vector3d &output,
                      const Vector3d &limits,
                      const bool &proportional_limitation) {
  Vector3d limited_output = output;

  // Delimit the speed for each axis
  if (proportional_limitation) {
    for (short j = 0; j < 3; j++) {
      if (limits[j] == 0.0f || output[j] == 0.0) {
        continue;
      };

      if (limited_output[j] > limits[j] || limited_output[j] < -limits[j]) {
        limited_output *= std::abs(limits[j] / limited_output[j]);
      }
    }
  } else {
    for (short j = 0; j < 3; j++) {
      if (limits[j] == 0.0f) {
        continue;
      }
      limited_output[j] = (limited_output[j] > limits[j]) ? limits[j] : limited_output[j];
      limited_output[j] = (limited_output[j] < -limits[j]) ? -limits[j] : limited_output[j];
    }
  }
  return limited_output;
}

static void BM_TEST_LIMITOUTPUT1(benchmark::State &state) {
  Vector3d output              = 2 * Vector3d::Ones();
  Vector3d limits              = Vector3d::Ones();
  bool proportional_limitation = true;
  for (auto _ : state) {
    limitOutput1(output, limits, proportional_limitation);
  }
}
BENCHMARK(BM_TEST_LIMITOUTPUT1)->Threads(1)->Repetitions(20);

static void BM_TEST_LIMITOUTPUT2(benchmark::State &state) {
  Vector3d output              = 2 * Vector3d::Ones();
  Vector3d limits              = Vector3d::Ones();
  bool proportional_limitation = true;
  for (auto _ : state) {
    limitOutput2(output, limits, proportional_limitation);
  }
}
BENCHMARK(BM_TEST_LIMITOUTPUT2)->Threads(1)->Repetitions(20);

int main(int argc, char **argv) {
  // benchmark::RegisterBenchmark("run Efficiciency", BM_TEST);
  benchmark::Initialize(&argc, argv);
  benchmark::RunSpecifiedBenchmarks();
  benchmark::Shutdown();
}