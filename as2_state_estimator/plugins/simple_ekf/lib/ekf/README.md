# ekf

Standalone 15-state Extended Kalman Filter for IMU-driven pose estimation.

The filter is defined symbolically in Python with [CasADi](https://web.casadi.org/), which
generates the state transition function, its Jacobians and the Kalman update equations as C
code. That generated code is committed to the repository and compiled into a small C++
library whose only runtime dependency is Eigen.

This library knows nothing about ROS. It is consumed by the
[`simple_ekf`](../../README.md) plugin, which does all the ROS work: subscriptions,
frames, timing and TF.

```mermaid
graph LR
  A["ekf.py<br/>symbolic model (CasADi)"] -->|c_code_generator.py| B["ekf_c_code.cpp/.h<br/>generated, committed"]
  B --> C["libekf.so<br/>EKFWrapper"]
  C --> D["simple_ekf plugin"]
```

## The filter

### State

| Index | Symbol | Meaning |
| --- | --- | --- |
| 0-2 | `x, y, z` | Position |
| 3-5 | `vx, vy, vz` | Velocity |
| 6-8 | `roll, pitch, yaw` | Orientation, Euler angles |
| 9-11 | `abx, aby, abz` | Accelerometer bias |
| 12-14 | `wbx, wby, wbz` | Gyroscope bias |

Named constants for these indices are in `ekf::State` and `ekf::Covariance`
([`include/ekf/ekf_datatype.hpp`](include/ekf/ekf_datatype.hpp)).

### Prediction

Input is the raw IMU: three accelerations and three angular rates. Biases and noise are
subtracted, then the continuous dynamics

```
p_dot = v
v_dot = R(rpy) * (a_meas - b_a) + g
rpy_dot = E(rpy) * (w_meas - b_w)
b_dot   = 0
```

are integrated with a 4th-order Runge-Kutta step. Biases are modelled as constants driven
only by their random walk in the process noise.

Process noise covariance is built from four scalars: accelerometer and gyroscope noise
density, and accelerometer and gyroscope random walk. The position/velocity block uses the
standard `dt³/3`, `dt²/2`, `dt` structure.

### Correction

Two measurement models:

| Model | Measures | Function |
| --- | --- | --- |
| Pose | `x, y, z, roll, pitch, yaw` | `update_pose()`, `update_pose_odom()` |
| Velocity | `vx, vy, vz` | `update_velocity()` |

Both are linear in the state, so the Jacobians are constant selection matrices. Measurement
noise is diagonal, given as a per-component variance vector.

`update_pose()` and `update_pose_odom()` share the same maths. They differ in bookkeeping:
`update_pose()` also advances the internal `map -> odom` transform and its velocity, so the
correction is expressed as a shift of the global frame, while `update_pose_odom()` leaves
`map -> odom` alone and lets the correction be absorbed downstream. This is what the
plugin's `is_odometry` flag selects between.

## C++ API

```cpp
#include <ekf/ekf_wrapper.hpp>

ekf::EKFWrapper filter;

std::array<double, ekf::Covariance::size> p0;
p0.fill(0.0);
p0[ekf::Covariance::ABX] = 1e-8;  // ... and the other bias terms
filter.reset(ekf::State(), ekf::Covariance(p0));

filter.set_gravity(ekf::Gravity(std::array<double, ekf::Gravity::size>({0.0, 0.0, 9.81})));
filter.set_noise_parameters(imu_noise, acc_nd, gyro_nd, acc_rw, gyro_rw);

// Prediction, once per IMU sample
ekf::Input imu_input;
imu_input.set({ax, ay, az, wx, wy, wz});
filter.predict(imu_input, dt);

// Correction from an absolute pose
ekf::PoseMeasurement z;
z.set({x, y, z_pos, roll, pitch, yaw});
ekf::PoseMeasurementCovariance r;
r.set({1e-4, 1e-4, 1e-4, 1e-5, 1e-5, 1e-5});
filter.update_pose(z, r);

ekf::State state = filter.get_state();
auto position = state.get_position();
auto quaternion = state.get_orientation_quaternion();
```

Useful extras:

| Method | Purpose |
| --- | --- |
| `correct_state()` | Wrap Euler angles back into `[-pi, pi]` |
| `get_map_to_odom()` | Accumulated global correction, 4×4 matrix |
| `get_map_to_odom_velocity()` | Its velocity counterpart |
| `pose_to_transform()`, `transform_to_pose()` | Conversions between a 4×4 matrix and position + Euler angles |
| `projectToSO3()` | Re-orthonormalise a rotation matrix that has drifted numerically |
| `to_string()`, `to_string_diagonal()` | Human-readable state and covariance dumps for debugging |

`State`, `Covariance`, `Input`, `Gravity` and the measurement types are plain
`std::array`-backed structs with named index constants, so they are cheap to copy. That is
what lets the plugin's history buffer snapshot the whole filter for out-of-sequence replay.

## Layout

| Path | Contents |
| --- | --- |
| `include/ekf/ekf_wrapper.hpp` | The filter class, the public API |
| `include/ekf/ekf_datatype.hpp` | `State`, `Covariance`, `Input`, `Gravity`, measurement types |
| `include/ekf/ekf_c_code.h`, `src/ekf_c_code.cpp` | **Generated.** Do not edit |
| `src/ekf_wrapper.cpp`, `src/ekf_datatype.cpp` | Hand-written implementation |
| `ekf_definition/ekf.py` | The symbolic model. This is the source of truth for the maths |
| `ekf_definition/casadi_utils.py` | Symbolic rotation and derivative helpers |
| `ekf_definition/ekf_wrapper.py` | Python equivalent of the C++ wrapper |
| `ekf_definition/transform_utils.py` | NumPy pose and quaternion helpers |
| `ekf_definition/ros_ekf.py` | Standalone Python ROS prototype node. Reference only, not built or installed |
| `ekf_definition/test/ekf/test_ekf.py` | Python tests of the symbolic filter |
| `scripts/c_code_generator.py` | Regenerates the C code from the model |
| `examples/simple_example.cpp` | Minimal C++ usage |

## Regenerating the C code

Only needed when `ekf_definition/ekf.py` changes. The generated files are committed, so a
normal build never runs CasADi.

```bash
cd plugins/simple_ekf/lib/ekf/scripts
python3 c_code_generator.py
```

It writes `ekf_c_code.cpp` and `ekf_c_code.h` into `../src/` and `../include/ekf/`,
prompting before overwriting. Requires `casadi` in the Python environment; the script puts
the library root on `sys.path` itself, so no installation step is needed.

Afterwards, check that `ekf::State` and `ekf::Covariance` still match the model's state
layout, and rebuild.

The generated files are excluded from the package's lint tests, since they are machine
output.

## Building and testing

The library is pulled in by the plugin via `add_subdirectory(lib/ekf)`, so building
`as2_state_estimator` builds it. Its only dependency is Eigen3, which makes it
straightforward to lift out and use elsewhere.

C++ examples are off by default:

```bash
colcon build --packages-select as2_state_estimator --cmake-args -DBUILD_EXAMPLES=ON
```

The C++ tests for this library live with the plugin, in
[`plugins/simple_ekf/tests`](../../tests) (`ekf_wrapper_gtest.cpp` covers the filter maths,
`ekf_history_buffer_gtest.cpp` covers rewind and replay), and run with the package's normal
`colcon test`.

The Python tests are not wired into ament and are run directly:

```bash
cd plugins/simple_ekf/lib/ekf
python3 -m pytest ekf_definition/test
```

## Known limitations

- **Euler angle attitude** is singular at pitch = ±90°. Adequate for normal flight, not for
  aggressive or near-vertical manoeuvres. A quaternion formulation would keep the same
  structure and measurement models while removing the failure mode; the quaternion helpers
  in `casadi_utils.py` are already there for it.
- **The Kalman gain uses a pseudo-inverse** (`ca.pinv`) rather than a Cholesky solve. Safe
  against a singular innovation covariance, but not the fastest option.
- **The covariance update is the plain `(I - KH) P` form**, which is not guaranteed to stay
  symmetric under accumulated floating-point error. The Joseph form would be more robust.
