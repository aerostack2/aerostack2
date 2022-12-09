#include <benchmark/benchmark.h>
#include <exception>

#include <math.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>

using Vector3d = Eigen::Vector3d;
using Matrix3d = Eigen::Matrix3d;

Vector3d filtered = Vector3d::Ones();
double alpha_1    = 1.0;
Matrix3d alpha_2  = Matrix3d::Identity();
Vector3d alpha_3  = Vector3d::Ones();

Vector3d computeFilter1(Vector3d &new_value) {
  return filtered = alpha_1 * new_value + (1.0 - alpha_1) * filtered;
}

Vector3d computeFilter2(Vector3d &new_value) {
  return filtered = alpha_2 * new_value + (Matrix3d::Identity() - alpha_2) * filtered;
}

Vector3d computeFilter3(Vector3d &new_value) {
  return filtered =
             alpha_3.cwiseProduct(new_value) + (Vector3d::Ones() - alpha_3).cwiseProduct(filtered);
}

static void BM_TEST_FILTER1(benchmark::State &state) {
  Vector3d value = Vector3d::Ones();
  filtered       = Vector3d::Ones();

  for (auto _ : state) {
    computeFilter1(value);
  }
}
BENCHMARK(BM_TEST_FILTER1)->Threads(1)->Repetitions(20);

static void BM_TEST_FILTER2(benchmark::State &state) {
  Vector3d value = Vector3d::Ones();
  filtered       = Vector3d::Ones();
  for (auto _ : state) {
    computeFilter2(value);
  }
}
BENCHMARK(BM_TEST_FILTER2)->Threads(1)->Repetitions(20);

static void BM_TEST_FILTER3(benchmark::State &state) {
  Vector3d value = Vector3d::Ones();
  filtered       = Vector3d::Ones();
  for (auto _ : state) {
    computeFilter3(value);
  }
}
BENCHMARK(BM_TEST_FILTER3)->Threads(1)->Repetitions(20);

int main(int argc, char **argv) {
  // benchmark::RegisterBenchmark("run Efficiciency", BM_TEST);
  benchmark::Initialize(&argc, argv);
  benchmark::RunSpecifiedBenchmarks();
  benchmark::Shutdown();
}