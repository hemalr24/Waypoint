# Stage 3: encoder and gyro fusion

An original, small C++ planar extended Kalman filter is implemented in `include/route_robot/ekf.hpp` and `src/ekf.cpp`. This is an educational estimator for the portable harness, not an integration of ROS `robot_localization`. The controller can now select `ground_truth`, `odometry`, or `ekf` feedback.

## Reproduce

```bash
make test
make fusion
python3 scripts/compare_feedback.py --stage 3 --routes circle --scenarios radius_mismatch --seeds 11 --animate --output results/fusion_demo
./build/simulate circle radius_mismatch results/replay ekf 11
```

The default comparison uses all three routes, all nine scenarios, all three feedback modes, and seeds 11, 22, 33, 44, 55: **405 simulations**. All scenarios now contain gyro noise, so every seed is used even for nominal conditions. The ground-truth and odometry controllers ignore the gyro for control but retain matching trials for comparison. `--no-plots` saves time when only measurements are needed. Choose a separate output directory for a new experiment.

## Sensors and timing

Wheel increments retain the Stage 2 shaft/ground distinction and noise streams. The gyro measures physical body turn rate:

```text
gyro_rate = (effective_right_ground_speed − effective_left_ground_speed) / actual_track_width
            + fixed_bias + Gaussian_noise
```

Gyro noise has standard deviation 0.02 rad/s at 50 Hz. An independent RNG stream uses `seed XOR 0x9e3779b9`, preserving the original encoder draws. Each feedback mode receives matching wheel and gyro noise samples over the common simulation interval. Different control actions can produce different physical rates; only noise and scenario settings are paired.

At time t, sensor columns describe the just-finished interval `[t-dt, t]`; the EKF has processed those samples before its pose is supplied to control. Commands recorded at t apply to `[t, t+dt]`. The initial row has no sensor sample: sensor and innovation fields are zero, and both estimators start at the known initial pose.

Only the sensor generator reads the simulated physical rate. The EKF receives three scalar rate observations and dt. It never receives simulator position, true heading, odometry pose, path, or controller commands. IMU acceleration, absolute orientation, magnetometer data, and an absolute position reference are absent.

## Filter model

The state is `[x, y, theta, v, omega]`, with pose in the world frame and forward/turn rates in the body frame. Prediction assumes constant v and omega over each interval and uses a midpoint approximation:

```text
direction = theta + omega × dt / 2
x_next = x + v × dt × cos(direction)
y_next = y + v × dt × sin(direction)
theta_next = wrap(theta + omega × dt)
v_next = v
omega_next = omega
P_predicted = F P Fᵀ + Q
```

F is the analytic state Jacobian; a finite-difference test verifies it. The plant integrates exact constant-curvature arcs, so estimator and plant do not share the same motion implementation. The midpoint approximation and interval-average rate measurements are intended for the fixed 0.02 s cadence. Sharp changes in actual velocity violate the constant-rate assumption and can introduce transient lag.

Process noise models independent, piecewise constant linear and angular accelerations. With `a = direction`:

```text
g_linear  = [0.5 dt² cos(a), 0.5 dt² sin(a), 0, dt, 0]
g_angular = [0, 0, 0.5 dt², 0, dt]
Q = sigma_accel² g_linear g_linearᵀ + sigma_angular_accel² g_angular g_angularᵀ
```

This is a short-step approximation; higher-order positional effects of angular acceleration are omitted. Prediction is followed by sequential scalar measurement updates for wheel forward speed, wheel turn rate, and gyro turn rate. Each observes state component v or omega. The covariance uses the Joseph update:

```text
innovation = measurement − predicted_component
S = P[component, component] + observation_variance
K = P[:, component] / S
state = state + K × innovation
A = I − K H
P = A P Aᵀ + K R Kᵀ
```

Heading is wrapped after prediction and correction, and covariance is symmetrized after correction. No dense matrix inversion is needed for scalar observations. Invalid/nonfinite observations and nonpositive dt are rejected before changing filter state.

The equal assumed wheel radii and equal, independent per-wheel noise make encoder forward/turn-rate noise uncorrelated in the nominal measurement model. Covariance tuning additionally allows wheel-model error. Persistent radius mismatch and slip violate the zero-mean independent-noise assumptions, so modeled covariance is not proof of accuracy. The implementation does not fuse encoder-derived pose as additional evidence: that would reuse the same information. This follows the sensor-selection principle in the [official robot_localization configuration guide](https://github.com/cra-ros-pkg/robot_localization/blob/rolling-devel/doc/configuring_robot_localization.rst).

## Fixed tuning

The following configuration is used unchanged across every route and scenario. Actual disturbance parameters are not used to adjust filter confidence.

| Parameter | Value |
|---|---:|
| Wheel forward-speed observation standard deviation | 0.04 m/s |
| Wheel turn-rate observation standard deviation | 0.25 rad/s |
| Gyro observation standard deviation | 0.02 rad/s |
| Per-interval linear-acceleration standard deviation | 1.0 m/s² |
| Per-interval angular-acceleration standard deviation | 4.0 rad/s² |
| Initial x/y variance | 1e-9 m² |
| Initial heading variance | 1e-9 rad² |
| Initial forward-speed variance | 1 (m/s)² |
| Initial turn-rate variance | 1 (rad/s)² |

Initial velocity means are zero. Process and observation settings, initial covariance, physical gyro noise, and injected bias are recorded in each `run.json`. The gyro is trusted more than wheel turn rate under this tuning. That choice can help with asymmetric wheel errors and hurt when the gyro is biased.

## Experiments and failure cases

All seven Stage 2 scenarios remain. Two more expose limitations:

- `gyro_bias`: adds +0.08 rad/s to the gyro with otherwise ideal wheels. The filter does not model bias, so an unbiased wheel-odometry estimate can outperform fusion.
- `symmetric_slip`: multiplies both effective wheel ground speeds by 0.7 during `[2, 8)` seconds while encoders continue to report shaft rotation. In straight motion there is no yaw-rate discrepancy to reveal the missing forward displacement.

The Stage 2 success criteria are unchanged: controller completion by 60 s, final true endpoint error at most 0.15 m, at least 95% true route progress, and maximum true cross-track error at most 1 m. Heading error is measured but is not a success criterion. Failures stay in the statistics. A successful estimated stop still need not mean physical success.

The gyro can constrain angular-rate error, but it supplies neither absolute heading nor position. Bias accumulates through integration, and a common error in forward displacement cannot generally be removed with a gyro. No claim of eliminating long-term drift is made.

## Two distinct comparisons

`paired.csv` compares separate controlled runs with the same route/scenario/seed. It includes odometry minus ground truth, EKF minus ground truth, and EKF minus odometry. Positive error differences mean the selected mode performed worse. These trajectories and durations can differ because the controller responds to different estimates.

`same_run_estimators.csv` compares passive odometry and EKF estimates on the **same physical trajectory, sensor samples, and duration**. Both estimators execute on every run, regardless of which supplies the controller. The `trajectory_feedback` column identifies that run's driving controller. This comparison isolates estimator behavior from changed control trajectories. Negative `delta_rms_m` means the EKF reduced position-localization RMS for that particular history.

The overview plot and report show true tracking error, controller localization error, and evaluated success. First-seed plots are chosen by configuration rather than outcome. Aggregate whiskers show observed min/max, not confidence intervals. Five seeds provide a modest simulation comparison, not a general reliability claim.

## Diagnostics and validation

Trajectory logs add the full EKF state, gyro measurement/noise, gyro innovation and innovation variance, five covariance diagonal entries, and per-update EKF execution time. Innovation is recorded after the wheel updates and before the gyro update. The zero initial innovation variance is a no-sample sentinel. Covariance diagonals have squared state units. EKF timing excludes sensor generation, odometry, control, and logging; it is not an end-to-end deadline guarantee.

C++ tests check the nonlinear Jacobian against numerical derivatives, stationary and constant-curvature motion, a scalar Gaussian posterior calculated independently, covariance symmetry/positive definiteness over long and varying-motion runs, weighting, wrapping, and input rejection. Python integration tests check sensor timestamps, physical gyro generation, repeatability, shared noise streams, absence of true-position leakage, a heading-improvement fixture, bias/slip failures, and full report generation.

ROS 2/Gazebo integration remains pending. The next integration milestone is to reproduce these comparisons using an existing robot and estimator stack with explicit timestamps and coordinate frames before considering optional MPC.
