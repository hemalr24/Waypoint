# Mobile Robot Path Tracking and State Estimation

A C++ pure-pursuit controller and reproducible Python evaluation pipeline for a differential-drive mobile robot. The portable kinematic harness supports a **Stage 1 ground-truth baseline** and **Stage 2 wheel-odometry feedback with controlled disturbances**.

The harness makes the controller runnable without ROS 2 or Gazebo. It uses commanded shaft velocities, exact planar kinematics, synthetic encoder measurements, and prescribed slip factors; it does not simulate rigid-body dynamics, a caster, or contact forces. Integrating an existing robot model with ROS 2 control and Gazebo remains the next integration milestone. IMU fusion and MPC are planned extensions.

## Run

Requires a C++17 compiler, Make, Python 3, NumPy, Matplotlib, and Pillow. With those dependencies available:

```bash
make
make test
make demo
```

Run the Stage 2 feedback comparison:

```bash
make compare
```

This runs 90 simulations across three routes and seven scenarios, using five paired seeds for each noisy scenario and one run per mode for each deterministic scenario. It writes `results/stage2/report.md`, `comparison.png`, individual logs, and aggregate metrics. Per-run plots show the first listed seed for each route/scenario/mode.

For a shorter comparison with animated slip behavior:

```bash
python3 scripts/compare_feedback.py --routes circle --scenarios wheel_slip --animate --output results/slip_demo
```

Use `--seeds 11 22 33` to set the noisy-trial seed list or `--no-plots` to generate only measurements and reports. See [Stage 2 experiment details](docs/stage2.md) for sensor assumptions, success criteria, and output fields.

To install Python dependencies into a virtual environment if needed:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
```

`make demo` runs all nine route/scenario combinations, creates comparison metrics and plots, and animates the three nominal routes. Rendering the GIFs takes longer than simulation. For faster runs:

```bash
python3 scripts/run_experiments.py
python3 scripts/run_experiments.py --routes circle --scenarios nominal offset --animate --output results/circle_comparison
```

Run a single simulation without plotting:

```bash
./build/simulate s_curve offset results/single_run
./build/simulate circle combined results/single_odometry_run odometry 22
```

Generated outputs:

- `results/report.md`: readable comparison and completion rates.
- `results/summary.csv`: all measured metrics.
- `results/manifest.json`: feedback mode, scenario definitions, and fixed configuration.
- `results/<route>_<scenario>/path.csv`: desired route points.
- `results/<route>_<scenario>/trajectory.csv`: timestamps, truth, supplied estimate, target, body and wheel commands, actual wheel speeds, progress, error, timing, and termination flags.
- `results/<route>_<scenario>/metrics.json`: metrics for one run.
- `results/<route>_<scenario>/tracking.png`: route, actual and estimated trajectories, target, tracking error, and commanded speed.
- `results/<route>_nominal/tracking.gif`: animation with robot heading and synchronized time cursors when `--animate` is set. Playback is condensed; timestamps show simulation time.

Each invocation overwrites artifacts for matching runs in its output directory. Its summary and manifest describe only the selected runs; choose a fresh `--output` for separate comparisons.

## Controller

`include/route_robot/controller.hpp` exposes a library interface independent of middleware. `PurePursuit::update(pose, path)` returns forward and angular speed, target point, progress, and completion status. Use one controller per traversal and call `reset()` before starting a new path.

The controller projects the supplied position onto a bounded forward section of the polyline. Progress is monotonic, and the target is interpolated one lookahead distance along the path. In body coordinates, with target offset `(x_b, y_b)`, curvature is:

```text
curvature = 2 y_b / (x_b² + y_b²)
angular_speed = forward_speed × curvature
left_wheel_speed  = forward_speed − angular_speed × track_width / 2
right_wheel_speed = forward_speed + angular_speed × track_width / 2
```

Forward speed is reduced to respect the angular limit, then both commands are scaled together to respect wheel limits. If the target is behind the robot, it turns in place. Near the endpoint, forward speed decreases with endpoint distance. Completion requires both sufficient route progress and position within 6 cm of the endpoint, then commands become zero. A circle is a finite single traversal; coincident endpoints do not cause immediate completion.

The controller does not regulate final heading independently. Heading error is measured, so this limitation remains visible. The local progress rule assumes initialization near the beginning of a route and frequent updates; arbitrary teleportation, large off-route starts, and self-intersecting routes need additional recovery logic. Commands have speed limits but no acceleration limits.

| Setting | Value |
|---|---:|
| Update period | 0.02 s (50 Hz) |
| Lookahead | 0.5 m |
| Cruise speed | 0.6 m/s |
| Angular speed limit | 1.8 rad/s |
| Track width | 0.36 m |
| Wheel rim speed limit | 0.9 m/s |
| Position tolerance | 0.06 m |
| Approach gain | 1.2 /s |
| Run timeout | 60 s |

## Stage 1 experiments and interpretation

Routes are a 6 m straight line, a circle of radius 2 m, and an 8 m S-curve with 1 m amplitude. Each is sampled as 400 segments. `Path` also accepts user-defined waypoint vectors in C++.

| Scenario | Initial condition / disturbance | Purpose |
|---|---|---|
| `nominal` | First route point, tangent heading | Basic tracking |
| `offset` | Add 0.5 m in world y and 0.6 rad to nominal heading | Recovery from a known pose offset |
| `stalled` | Both wheels stop moving after 2 s, even while commanded | Explicit actuator-failure and timeout case |

All scenarios are deterministic, so there are no random seeds or repeated stochastic trials at this stage. Timing varies with the machine and load. The stalled scenario is an artificial failure fixture, not a wheel-slip model.

Cross-track error is the nonnegative shortest Euclidean distance from true position to any segment of the desired finite polyline, including clamped endpoints. RMS and maximum error include every logged sample, including the initial pose and terminal pose, at a uniform 50 Hz cadence. Final position error is distance to the final waypoint. Final heading error is the wrapped true heading minus the last segment's direction. Localization RMS compares the supplied estimate with truth; it is exactly zero by construction in these Stage 1 runs.

Completion is a controller-reported positional stop before the timeout; it does not require final heading alignment. The summary reports completion fractions across selected routes for each scenario. These deterministic fractions are not estimates of stochastic reliability. Controller computation time measures only `update()`, excluding simulation, logging, middleware, and sensor handling; it is not an end-to-end deadline guarantee.

Expect small errors when the finite lookahead smooths curved paths, and larger transient errors after an initial offset. The controller steers to reduce lateral target displacement, then resumes forward travel once aligned. A stalled robot cannot follow the route even with perfect localization. On a straight route it can report zero cross-track error while failing to finish, which demonstrates why completion and final position error matter alongside tracking error.

## Next stages

See [the integration and experiment roadmap](docs/roadmap.md) for ROS 2/Gazebo integration, sensor fusion, and optional MPC. Stage 2 odometry comparisons currently run in the portable harness; the external simulator integration is still pending.
