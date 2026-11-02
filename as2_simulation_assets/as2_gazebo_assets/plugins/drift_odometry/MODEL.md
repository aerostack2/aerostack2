# Drift odometry model

This page explains the math in [`model.hpp`](model.hpp), the model behind the `DriftOdometryPublisher` Gazebo
plugin. It is meant for contributors who want to change the model. To use the plugin, see the
[drifting odometry section](../../README.md#drifting-odometry) of the package README.

The model turns ground-truth poses into odometry that drifts like VIO or LiDAR odometry. Only x, y, z and yaw drift,
because those are the directions such systems cannot observe. Roll and pitch stay exact, since gravity makes them
observable.

## The step at a glance

At each ground-truth sample $`k`$, `propagate()` advances the drifted estimate from the previous sample $`k-1`$.
Indices follow one rule:

- **State** carries the sample it belongs to: $`k-1`$ on the right-hand side (the values on entry, including
  `last_true_pose_`), and $`k`$ on the left-hand side (the values on exit, and `true_pose`).
- **Quantities of the step** carry $`k`$, the sample the step ends at: $`\Delta t_k`$, $`\Delta\mathbf p_k`$,
  $`\mathbf h_k`$, $`\mathbf w_k`$, $`F_k`$, $`Q_k`$ and the noise draws.

```math
\begin{aligned}
\mathbf h_k &= s\,R_z(\psi_{k-1})^{\top}\Delta\mathbf p_k + \mathbf b_{k-1}\,\Delta t_k + \sigma_v\sqrt{\Delta t_k}\,\mathbf n_k
  && \text{measure and corrupt the step}\\
\mathbf w_k &= R_z(\psi_{k-1} + e_{k-1})\,\mathbf h_k
  && \text{rotate it back with the estimated yaw}\\
\hat{\mathbf p}_k &= M(\hat{\mathbf p}_{k-1} + \mathbf w_k) + (I - M)\,\mathbf p_k
  && \text{axes without drift follow the truth}\\
P_k &= S\,(F_k P_{k-1} F_k^\top + Q_k)\,S
  && \text{covariance}\\
\mathbf b_k &= \mathbf b_{k-1} + M\,\sigma_b\sqrt{\Delta t_k}\,\mathbf m_k
  && \text{velocity bias random walk}\\
e_k &= e_{k-1} + \beta_{k-1}\,\Delta t_k + \sigma_\psi\sqrt{\Delta t_k}\,u_k,
\quad \beta_k = \beta_{k-1} + \sigma_\beta\sqrt{\Delta t_k}\,\eta_k
  && \text{yaw random walks}
\end{aligned}
```

Each line is explained in the sections below. Before drift starts (`drift_start: takeoff`), `update()` re-initializes
on every sample instead, so the estimate equals ground truth and all biases and covariances are zero.

## Symbols

| Symbol | Code | Meaning |
| - | - | - |
| $`\Delta t_k`$ | `dt` | Time between samples, $`t_k - t_{k-1}`$. It may vary from step to step |
| $`\mathbf p_{k-1},\ \psi_{k-1}`$ | `last_true_pose_` | True position and yaw at the previous sample |
| $`\mathbf p_k`$ | `true_pose.position` | True position at the current sample |
| $`\Delta\mathbf p_k`$ | `delta` | True displacement $`\mathbf p_k - \mathbf p_{k-1}`$, world frame |
| $`\hat{\mathbf p}_k`$ | `estimated_position_` | Drifted position estimate |
| $`e_k`$ | `yaw_error_` | Yaw error; the estimated yaw is $`\hat\psi_k = \psi_k + e_k`$ |
| $`\mathbf b_k \in \mathbb R^3`$ | `velocity_bias_` | Velocity bias, in the heading frame |
| $`\beta_k`$ | `yaw_rate_bias_` | Yaw rate bias |
| $`R_z(\psi_{k-1}),\ R_z(\hat\psi_{k-1})`$ | `r_true`, `r_est` | Heading to world rotations, with the true and the estimated yaw |
| $`\mathbf h_k`$ | `measured` | Corrupted step, heading frame |
| $`\mathbf w_k`$ | `step` | Corrupted step, odometry frame |
| $`P_k`$ | `covariance_` | 8×8 covariance of the error state |
| $`s`$ | `scale` | $`1 +`$ `scale_error` |
| $`M`$ | `drift_horizontal`, `drift_vertical` | Axis mask $`\mathrm{diag}(m_h, m_h, m_v)`$, where $`m = 1`$ if the axis drifts and 0 otherwise |
| $`m_\psi`$ | `drift_yaw` | 1 if yaw drifts, 0 otherwise |
| $`\sigma_v,\ \sigma_b`$ | `linear_velocity_noise`, `linear_bias_noise` | Noise densities for position and velocity bias |
| $`\sigma_\psi,\ \sigma_\beta`$ | `yaw_rate_noise`, `yaw_rate_bias_noise` | Noise densities for yaw and yaw rate bias |
| $`\mathbf n_k,\ \mathbf m_k \in \mathbb R^3`$ | `gaussian3()` | Independent $`\mathcal N(\mathbf 0, I_3)`$ draws, in fixed x, y, z order |
| $`u_k,\ \eta_k`$ | `gaussian()` | Independent $`\mathcal N(0, 1)`$ draws |

The members of `DriftOdometryModel` hold the $`k-1`$ values when `propagate()` starts and the $`k`$ values when it
returns.

All noise parameters are standard deviations. The four densities are the standard deviation after 1 s; section 8
explains why the noise is scaled by √Δt.

## 1. Yaw and the two rotations

The only rotation the model uses is a yaw rotation about z. Its derivative uses $`[\hat{\mathbf z}]_\times`$, which
turns a vector 90° to the left in the horizontal plane:

```math
R_z(\theta) = \begin{bmatrix}\cos\theta & -\sin\theta & 0\\ \sin\theta & \cos\theta & 0\\ 0 & 0 & 1\end{bmatrix},
\qquad
[\hat{\mathbf z}]_\times = \begin{bmatrix}0 & -1 & 0\\ 1 & 0 & 0\\ 0 & 0 & 0\end{bmatrix},
\qquad
\frac{d}{d\theta}R_z(\theta) = [\hat{\mathbf z}]_\times R_z(\theta).
```

So $`\hat{\mathbf z}\times\mathbf v = (-v_y,\ v_x,\ 0)`$. Rotations about the same axis add:
$`R_z(a)\,R_z(b)^\top = R_z(a - b)`$.

`yawOf()` takes the yaw of the previous true orientation $`q_{k-1}`$, the z angle of its z-y-x Euler angles:

```math
\psi_{k-1} = \mathrm{atan2}\big(2(q_w q_z + q_x q_y),\ q_w^2 + q_x^2 - q_y^2 - q_z^2\big).
```

The **heading frame** is the vehicle's frame with roll and pitch removed: x forward, y left, z up. Two rotations take
it to the world frame:

- $`R_z(\psi_{k-1})`$, with the **true** yaw, describes how the vehicle really moved.
- $`R_z(\hat\psi_{k-1}) = R_z(\psi_{k-1} + e_{k-1})`$, with the **estimated** yaw, describes how the estimator thinks
  it moved.

```cpp
const double true_yaw = yawOf(last_true_pose_.orientation);
const Eigen::Matrix3d r_true = yawRotation(true_yaw).toRotationMatrix();
const Eigen::Matrix3d r_est = yawRotation(true_yaw + yaw_error_).toRotationMatrix();
```

## 2. Measure the step in the heading frame and corrupt it

$`R_z(\psi_{k-1})^\top`$ undoes the true heading, so the step is expressed the way an odometry sensor sees it, for
example "1 m forward":

```math
\mathbf d_k = R_z(\psi_{k-1})^{\top} \Delta\mathbf p_k
= \begin{bmatrix} \cos\psi_{k-1}\,\Delta x + \sin\psi_{k-1}\,\Delta y \\ -\sin\psi_{k-1}\,\Delta x + \cos\psi_{k-1}\,\Delta y \\ \Delta z \end{bmatrix}.
```

$`R_z`$ leaves z unchanged, which is why one 3-vector expression handles horizontal and vertical motion together.
Three corruptions are then added in the heading frame:

```math
\mathbf h_k =
\underbrace{s\,\mathbf d_k}_{\text{scale error}}
+ \underbrace{\mathbf b_{k-1}\,\Delta t_k}_{\text{bias}}
+ \underbrace{\sigma_v \sqrt{\Delta t_k}\,\mathbf n_k}_{\text{white noise}},
\qquad \mathbf n_k \sim \mathcal N(\mathbf 0, I_3).
```

- **Scale** stretches every step by the same factor.
- **Bias** is a velocity tied to the vehicle (for example "always overestimates forward motion"), so it is multiplied
  by $`\Delta t_k`$ to become a distance.
- **White noise** has standard deviation $`\sigma_v\sqrt{\Delta t_k}`$, so its variance is $`\sigma_v^2\,\Delta t_k`$.

```cpp
const Eigen::Vector3d delta = true_pose.position - last_true_pose_.position;
const Eigen::Vector3d measured = scale * r_true.transpose() * delta +
  velocity_bias_ * dt + params_.linear_velocity_noise * sqrt_dt * gaussian3();
```

## 3. Rotate it back with the estimated yaw

```math
\mathbf w_k = R_z(\hat\psi_{k-1})\,\mathbf h_k,
\qquad
\hat{\mathbf p}_k = \hat{\mathbf p}_{k-1} + \mathbf w_k .
```

```cpp
const Eigen::Vector3d step = r_est * measured;
estimated_position_ += step;
```

> [!NOTE]
> **Why yaw error becomes position error.** Substituting $`\mathbf h_k`$ and using
> $`R_z(\psi_{k-1} + e_{k-1})\,R_z(\psi_{k-1})^\top = R_z(e_{k-1})`$:
>
> ```math
> \mathbf w_k = s\,R_z(e_{k-1})\,\Delta\mathbf p_k + R_z(\hat\psi_{k-1})\big(\mathbf b_{k-1}\Delta t_k + \sigma_v\sqrt{\Delta t_k}\,\mathbf n_k\big).
> ```
>
> Every true step is turned by the yaw error. For a small error,
> $`R_z(e_{k-1}) \approx I + e_{k-1}[\hat{\mathbf z}]_\times`$, so each step adds a sideways error
> $`e_{k-1}\,\hat{\mathbf z}\times\Delta\mathbf p_k`$. That error is proportional to the step length, so it grows with
> distance flown and is zero while hovering.

**Example.** The vehicle faces world +y ($`\psi = 90°`$) and flies 1 m forward, so
$`\Delta\mathbf p = (0, 1, 0)`$ and $`\mathbf d = R_z(90°)^\top\Delta\mathbf p = (1, 0, 0)`$, that is "1 m forward".
With no yaw error, $`R_z(90°)\,(1, 0, 0) = (0, 1, 0)`$, which matches the truth. With $`e = 5°`$,
$`R_z(95°)\,(1, 0, 0) = (-0.087,\ 0.996,\ 0)`$, so the step lands 8.7 cm to the side.

## 4. Axes without drift follow the truth

```cpp
if (!params_.drift_horizontal) {
  estimated_position_.head<2>() = true_pose.position.head<2>();
}
if (!params_.drift_vertical) {
  estimated_position_.z() = true_pose.position.z();
}
```

With the axis mask $`M = \mathrm{diag}(m_h, m_h, m_v)`$, the two branches together are

```math
\hat{\mathbf p}_k = M\,(\hat{\mathbf p}_{k-1} + \mathbf w_k) + (I - M)\,\mathbf p_k.
```

A drifting axis keeps the integrated estimate. An axis without drift is overwritten with ground truth, as with a GPS,
motion capture or barometer correction. The step and its noise are still computed for every axis, so each step uses
the same number of random draws whatever the settings.

## 5. Covariance

`propagateCovariance()` propagates the covariance of the error state, the difference between the estimate and the
truth. The true biases and the true yaw error are zero, so the bias and yaw states are their own errors:

```math
\delta\mathbf x = \big[\,\underbrace{\hat{\mathbf p}-\mathbf p}_{3},\ \underbrace{e}_{1},\ \underbrace{\mathbf b}_{3},\ \underbrace{\beta}_{1}\,\big]^\top \in \mathbb R^8 ,
\qquad
P_k = S\,\big(F_k\,P_{k-1}\,F_k^\top + Q_k\big)\,S .
```

**Jacobian $`F_k`$.** Differentiating $`\mathbf w_k = R_z(\psi_{k-1} + e_{k-1})\,\mathbf h_k`$ at the current values:

```math
\frac{\partial \mathbf w_k}{\partial e_{k-1}} = [\hat{\mathbf z}]_\times R_z(\hat\psi_{k-1})\,\mathbf h_k = \hat{\mathbf z}\times\mathbf w_k,
\qquad
\frac{\partial \mathbf w_k}{\partial \mathbf b_{k-1}} = R_z(\hat\psi_{k-1})\,\Delta t_k,
\qquad
\frac{\partial e_k}{\partial \beta_{k-1}} = \Delta t_k .
```

In block form, with the states ordered as $`[\delta\mathbf p,\ e,\ \mathbf b,\ \beta]`$:

```math
F_k =
\begin{bmatrix}
I_3 & \hat{\mathbf z}\times\mathbf w_k & R_z(\hat\psi_{k-1})\,\Delta t_k & \mathbf 0\\
\mathbf 0^\top & 1 & \mathbf 0^\top & \Delta t_k\\
0_3 & \mathbf 0 & I_3 & \mathbf 0\\
\mathbf 0^\top & 0 & \mathbf 0^\top & 1
\end{bmatrix}
```

These are the three block assignments in the code:

```cpp
f.block<3, 1>(kPx, kYaw) = Eigen::Vector3d::UnitZ().cross(step);
f.block<3, 3>(kPx, kBx) = r_est * dt;
f(kYaw, kBYaw) = dt;
```

<details>
<summary>F written out in full</summary>

With $`\hat c = \cos\hat\psi_{k-1}`$, $`\hat s = \sin\hat\psi_{k-1}`$ and $`\mathbf w = \mathbf w_k`$:

```math
F_k =
\left[\begin{array}{ccc|c|ccc|c}
1 & 0 & 0 & -w_y & \hat c\,\Delta t_k & -\hat s\,\Delta t_k & 0 & 0\\
0 & 1 & 0 & w_x & \hat s\,\Delta t_k & \hat c\,\Delta t_k & 0 & 0\\
0 & 0 & 1 & 0 & 0 & 0 & \Delta t_k & 0\\ \hline
0 & 0 & 0 & 1 & 0 & 0 & 0 & \Delta t_k\\ \hline
0 & 0 & 0 & 0 & 1 & 0 & 0 & 0\\
0 & 0 & 0 & 0 & 0 & 1 & 0 & 0\\
0 & 0 & 0 & 0 & 0 & 0 & 1 & 0\\ \hline
0 & 0 & 0 & 0 & 0 & 0 & 0 & 1
\end{array}\right]
```

</details>

**Process noise $`Q_k`$** is the variance each $`\sqrt{\Delta t_k}`$-scaled draw adds during the step:

```math
Q_k = \mathrm{diag}\big(\sigma_v^2\Delta t_k\, I_3,\ \ \sigma_\psi^2\Delta t_k,\ \ \sigma_b^2\Delta t_k\, I_3,\ \ \sigma_\beta^2\Delta t_k\big).
```

The position noise is drawn in the heading frame and then rotated, but $`\sigma_v^2\Delta t_k\,I_3`$ is isotropic, so
the rotation does not change it.

**Clearing $`S`$.** The `clear` lambda zeroes the rows and columns of every state on an axis without drift. That is
$`S = \mathrm{diag}(m_h, m_h, m_v, m_\psi, m_h, m_h, m_v, m_\psi)`$ applied on both sides. It also explains why a
nonzero $`\hat{\mathbf z}\times\mathbf w_k`$ is harmless when horizontal drift is off: those x and y rows are cleared
right after.

## 6. Random walks

After the position step, the biases and the yaw error advance:

```math
\begin{aligned}
\mathbf b_k &= \mathbf b_{k-1} + M\,\sigma_b\sqrt{\Delta t_k}\;\mathbf m_k \\
e_k &= e_{k-1} + m_\psi\big(\beta_{k-1}\,\Delta t_k + \sigma_\psi\sqrt{\Delta t_k}\;u_k\big) \\
\beta_k &= \beta_{k-1} + m_\psi\,\sigma_\beta\sqrt{\Delta t_k}\;\eta_k
\end{aligned}
```

The masks keep the biases at zero on axes without drift. Every right-hand side uses the $`k-1`$ values: the position
step used $`\mathbf b_{k-1}`$ and $`e_{k-1}`$, and the yaw update uses $`\beta_{k-1}`$ before $`\beta`$ advances. This
is the explicit (Euler) ordering.

Yaw has the same structure as position: white noise on the increment ($`\sigma_\psi`$, like $`\sigma_v`$) plus an
integrated bias ($`\beta`$, like $`\mathbf b`$).

Before advancing them, `propagate()` stores $`\mathbf b_{k-1}`$ and $`\beta_{k-1}`$ as `step_velocity_bias_` and
`step_yaw_rate_bias_`. These are the biases that acted during the step from $`t_{k-1}`$ to $`t_k`$. The velocities
reported at $`t_k`$ use them, so integrating the twist reproduces the bias part of the pose drift.

## 7. Reported pose and velocities

`makeOdometry()` reports the position $`\hat{\mathbf p}_k`$. For the orientation, it applies the yaw error to the true
orientation $`q_k`$ as a rotation about world z, so roll and pitch stay exact:

```math
\hat q_k = R_z(e_k) \otimes q_k .
```

The pose covariance is $`P_k`$ restricted to $`(x, y, z, \text{yaw})`$ and placed in ROS order
$`(x, y, z, r_x, r_y, r_z)`$. Its diagonal is floored at $`10^{-6}`$, so roll, pitch and axes without drift stay
positive definite.

`makeTwist()` splits the attitude into heading and tilt, $`q_k = R_z(\psi_k) \otimes T_k`$, where $`T_k`$ maps the
body frame to the heading frame. It then applies the same scale and bias as the pose, in the heading frame:

```math
\hat{\mathbf v}_k = T_k^{\top}\big( (sM + I - M)\,T_k\,\mathbf v_k + M\,\mathbf b_{k-1} \big) + \sigma_{tl}\,\mathbf n'_k,
\qquad
\hat{\boldsymbol\omega}_k = \boldsymbol\omega_k + m_\psi\,q_k^{-1}(0, 0, \beta_{k-1}) + \sigma_{ta}\,\mathbf n''_k,
```

where $`\mathbf v_k`$ and $`\boldsymbol\omega_k`$ are the true body-frame velocities, and $`\sigma_{tl}`$ and
$`\sigma_{ta}`$ are `twist_linear_noise` and `twist_angular_noise`. That noise is drawn fresh each sample and never
integrated into the pose. The twist covariance is the bias covariance rotated into the body frame, plus the per-sample
noise:

```math
\Sigma_{v,k} = T_k^{\top} P_k^{bb}\, T_k + \sigma_{tl}^2 I_3,
\qquad
\Sigma_{\omega,k} = P_k^{\beta\beta}\,\mathbf u_k\mathbf u_k^\top + \sigma_{ta}^2 I_3,
\qquad \mathbf u_k = q_k^{-1}\hat{\mathbf z}.
```

## 8. Why the noise is scaled by √Δt

The discrete steps are the Euler–Maruyama discretization of these stochastic differential equations:

```math
\begin{aligned}
d\hat{\mathbf p} &= s\,R_z(e)\,d\mathbf p + R_z(\hat\psi)\,\mathbf b\,dt + \sigma_v\,d\mathbf W_v \\
d\mathbf b &= \sigma_b\,d\mathbf W_b \\
de &= \beta\,dt + \sigma_\psi\,dW_\psi \\
d\beta &= \sigma_\beta\,dW_\beta
\end{aligned}
```

A Wiener process increment over $`\Delta t_k`$ is distributed as $`\mathcal N(0, \Delta t_k)`$, and
$`\sigma\sqrt{\Delta t_k}\,n`$ with $`n \sim \mathcal N(0, 1)`$ has exactly that distribution. The variance added per
step is $`\sigma^2\Delta t_k`$, so over any sequence of steps, even irregular ones, the variances add up to
$`\sigma^2 t`$. **The drift therefore does not depend on the update rate.** If the noise were added as $`\sigma\,n`$
per step, doubling the sample rate would double the variance.

> [!NOTE]
> **Random walk or Brownian motion?** Both. $`\mathbf b`$, $`\beta`$ and the white-noise parts of
> $`\hat{\mathbf p}`$ and $`e`$ are Brownian motion sampled exactly at the update times, which is a Gaussian random
> walk. The bias terms $`\int \mathbf b\,dt`$ and $`\int \beta\,dt`$ are integrated Brownian motion, and the code
> approximates those integrals with a left-rectangle sum.

## 9. How fast the error grows

For a hovering vehicle ($`\Delta\mathbf p = 0`$), one axis of position error is white noise plus an integrated random
walk. Using $`\mathrm{Var}\big(\int_0^t W(\tau)\,d\tau\big) = t^3/3`$:

```math
\mathrm{Var}\big(\delta p(t)\big) = \sigma_v^2\,t + \frac{\sigma_b^2\,t^3}{3},
\qquad
\mathrm{Var}\big(e(t)\big) = \sigma_\psi^2\,t + \frac{\sigma_\beta^2\,t^3}{3}.
```

The white-noise term grows as $`\sqrt t`$ in standard deviation, and the bias term grows as $`t^{3/2}`$. With the
defaults, the bias term takes over after about 35 s for position and 5 s for yaw. The covariance $`P_k`$ reproduces
these variances.

| 1σ at defaults, hovering | 10 s | 60 s | 300 s |
| - | -: | -: | -: |
| Position, white noise $`\sigma_v\sqrt t`$ | 0.063 m | 0.155 m | 0.346 m |
| Position, bias $`\sigma_b t^{3/2}/\sqrt 3`$ | 0.018 m | 0.268 m | 3.00 m |
| **Position, total (per axis)** | **0.066 m** | **0.310 m** | **3.02 m** |
| Yaw, white noise $`\sigma_\psi\sqrt t`$ | 0.36° | 0.89° | 1.98° |
| Yaw, bias $`\sigma_\beta t^{3/2}/\sqrt 3`$ | 0.05° | 0.77° | 8.59° |
| **Yaw, total** | **0.37°** | **1.17°** | **8.82°** |

In motion, yaw error adds a sideways error on top (section 3). Flying straight at speed $`v`$, the yaw white noise
alone contributes $`\mathrm{Var} \approx v^2\sigma_\psi^2 t^3/3`$.

## 10. Approximations and caveats

> [!WARNING]
> - **Scale error is not in $`P`$.** $`s`$ appears in neither $`F_k`$ nor $`Q_k`$, so a nonzero `scale_error` adds
>   an error, $`(s-1)(\mathbf p_k - \mathbf p_{\text{start}})`$ when there is no yaw error, that the covariance does not
>   report.
> - **Bias integrals use a left-rectangle sum.** $`\mathbf b_{k-1}\Delta t_k`$ is used where
>   $`\int_{t_{k-1}}^{t_k} \mathbf b\,dt`$ is meant. An exact discretization would add $`\sigma_b^2\Delta t_k^3/3`$ per
>   step and correlate the position and bias noise. At simulation rates this is negligible.
> - **The heading is held for the whole step.** The step is rotated with $`\psi_{k-1}`$, the yaw at the start of the
>   interval, which is accurate while the yaw changes little between samples.
> - **$`F_k`$ uses the noisy step $`\mathbf w_k`$.** The Jacobian is evaluated at the realized step, including its
>   noise, which is the usual EKF choice.
> - **The twist uses $`\mathbf b_{k-1}`$ but reports the covariance of $`\mathbf b_k`$.** The two differ by
>   $`\sigma_b^2\Delta t_k`$, which is negligible.
