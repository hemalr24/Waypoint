# Integration and experiment roadmap

## Stage 1 integration: existing simulator and robot

Choose a compatible ROS 2 / Gazebo release pair for the target machine and record the versions before adding dependencies. Reuse a small differential-drive model with two driven wheels and a caster, a flat world, and existing ROS 2 control components. Keep the standalone harness as a fast controller check.

Add a thin C++ ROS node around `PurePursuit`: accept a stamped state and path in a common frame, calculate commands on a simulation-time timer, and publish body velocity commands to the selected drive-controller interface. Convert the interface to the message type required by the chosen ROS 2 control version. The drive controller handles joint commands and wheel feedback. Match track width, wheel radii, velocity limits, and coordinate conventions with the model.

The node needs an explicit stop on completion, stale pose input, invalid data, shutdown, and route replacement. Use a command timeout at the drive-controller level as well. Reset traversal progress when accepting a new path. Check simulation clock handling and timestamp/frame consistency before tuning tracking parameters.

Initially supply simulator ground truth to the controller and label that mode in every output. Record ground truth separately for evaluation and preserve the standalone metrics schema. Validate straight, circle, and S-curve tracking plus offset recovery in the external simulator before treating the Stage 1 integration milestone as complete.

## Stage 2: wheel odometry and imperfect measurements

Implemented in the portable harness: a separate wheel-odometry estimator, encoder-velocity noise, wheel-radius mismatch, prescribed wheel slip, paired seed trials, truth-based success evaluation, and comparison artifacts. See [Stage 2 details](stage2.md). These experiments still need replication in the external simulator after the integration milestone above.

Integrate wheel angle increments using the assumed wheel radii and track width to produce planar pose. The controller must consume this estimate; ground truth is available only to the recorder. Independently vary encoder noise, wheel-radius mismatch, and wheel slip. Distinguish true wheel-ground motion from encoder rotation so that a slipping wheel can rotate without producing the expected displacement.

Use common routes, initial conditions, controller tuning, and predetermined seed lists for ground-truth-feedback and odometry-feedback comparisons. Store each seed and complete configuration. Start with one disturbance at a time, then add combined cases. Define failure before running trials: timeout, excessive deviation, or invalid state. Report distributions of tracking and localization errors plus completion rates, retaining failed trials.

## Stage 3: odometry and IMU fusion

A standalone five-state planar EKF is implemented in the portable harness, fusing wheel forward/turn rates with a simulated gyro. It includes covariance propagation, gyro-bias and symmetric-slip failure cases, paired controller comparisons, and passive estimator comparisons on identical sensor histories. See [Stage 3 details](stage3.md).

For ROS integration, compare this small educational filter with an existing planar estimator. Configure timestamps, frames, covariance, and sensor sources explicitly. Avoid counting correlated odometry-derived quantities as independent evidence. Replicate odometry-only and fused comparisons in the external simulator with the same scenarios and seeds. A future extension could estimate gyro bias, but wheel slip and gyro bias can be difficult to distinguish without another reference.

Wheel odometry and an IMU provide no absolute position reference. Measure whether fusion improves particular errors under particular disturbances; do not presume that it eliminates drift or always improves tracking. Examine heading bias and position drift over longer routes as well as short-run RMS metrics.

## Stage 4: optional MPC

Only begin after the controller integration and evaluation pipeline are stable. Add a finite-horizon planar model, speed/turning constraints, tracking and command-change costs, and explicit handling for solver failures and missed deadlines. Compare against pure pursuit with the same feedback source, paths, disturbances, seed lists, limits, and stopping criteria. Include computation-time distributions and failed runs, even when MPC does not improve the result.

## Scope

One simulated robot, a flat environment, and predefined routes. Physical hardware, obstacle avoidance, SLAM, reinforcement learning, and a custom dashboard are outside this project's initial scope.
