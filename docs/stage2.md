# Stage 2: wheel odometry and imperfect measurements

The controller can now use wheel odometry in place of ground truth. The estimator receives measured wheel angle increments and an initial pose. It has no simulator-state input after initialization. The recorder retains truth for evaluation. A separate ground-truth-feedback run serves as the baseline under the same physical conditions.

## Run and replay

```bash
make test
make compare
python3 scripts/compare_feedback.py --routes line circle --scenarios encoder_noise combined --seeds 11 22 33 --no-plots --output results/noisy_comparison
./build/simulate circle combined results/replay odometry 22
```

Each paired run shares route, starting pose, controller settings, physical disturbance schedule, and random seed. Encoder noise is drawn once per wheel per time step, in a fixed order in both modes. This gives identical noise samples over the shared time interval even when control actions differ. Scenarios with noise use every supplied seed; deterministic scenarios use only the first seed. Duplicate routes, scenarios, or seeds are rejected to prevent accidental double-counting.

Default seeds are 11, 22, 33, 44, and 55. There are three routes, two feedback modes, five deterministic scenarios, and two stochastic scenarios: 90 runs total. Outputs overwrite matching run directories; choose a fresh output directory to retain a separate experiment. Reports describe only the current invocation. Old plots can remain if you reuse a directory with `--no-plots`.

## Motion and sensor model

Body commands become left/right rim speeds using the assumed track width, then shaft angular velocities using the assumed 0.1 m wheel radius. The simulator multiplies shaft velocity by the actual wheel radius and a traction factor to produce effective ground speed. The body motion uses the actual 0.36 m track width.

Encoder measurements observe shaft rotation, before traction loss. For each wheel and sample:

```text
measured_angle_increment = (shaft_angular_velocity + velocity_noise) × dt
estimated_wheel_distance = assumed_wheel_radius × measured_angle_increment
estimated_translation = (right_distance + left_distance) / 2
estimated_rotation = (right_distance − left_distance) / assumed_track_width
```

Odometry integrates the resulting constant-curvature arc. It starts with the known true initial pose, including offsets; initial localization uncertainty is not modeled. `estimated_*` is the pose actually supplied to the controller. In ground-truth mode it equals truth; `odometry_*` independently records the encoder-only estimate in both modes.

| Scenario | Model | Random trials? |
|---|---|---|
| `nominal` | Exact geometry, no noise, full traction | No |
| `offset` | Known initial pose shifted +0.5 m in world y and +0.6 rad in heading | No |
| `encoder_noise` | Independent zero-mean Gaussian velocity noise, 0.4 rad/s standard deviation per wheel/sample | Yes |
| `radius_mismatch` | Actual left radius 0.098 m, right radius 0.102 m; assumed radii remain 0.1 m | No |
| `wheel_slip` | Effective left ground speed multiplied by 0.65, right by 0.95, during `[2, 8)` seconds | No |
| `combined` | Encoder noise, radius mismatch, and wheel slip together | Yes |
| `stalled` | Both shafts stop at 2 seconds; encoders also stop | No |

These are explicit kinematic disturbances, not a tire/ground contact model. Noise is white velocity noise sampled at a fixed 50 Hz, then integrated into angle increments; changing the sample period changes its accumulated uncertainty. Quantization, encoder bias, wheel inertia, sensor delays, IMU data, and terrain physics are outside this version.

## Independent success criteria

The controller stops using its supplied estimate, within its 0.06 m estimated goal tolerance. This is recorded as `complete` in the trajectory and `completed` in metrics. It is not proof of successful physical traversal.

The evaluator declares `success` only if all of these hold:

- The controller reports completion by the 60 s run limit.
- Final true position is within 0.15 m of the endpoint.
- True progress reaches at least 95% of the route length.
- Maximum true cross-track error throughout the run is at most 1.0 m.

These thresholds are declared before experiments in `CRITERIA` and stored in the manifest. The true-progress condition prevents a circle from appearing successful just because its endpoint coincides with its start. Progress uses a local monotonic projection onto the route, with the same 1 m forward search span as the baseline controller; it is an evaluation variable and never drives control. These progress rules are designed for the provided simple routes, not arbitrary self-intersecting paths.

Excessive deviation is classified after the run rather than using ground truth to interrupt odometry control. The simulator ends only on controller completion or timeout; both produce a terminal zero command. Final heading error is measured but is not a success criterion, because pure pursuit does not independently align endpoint heading.

Failed trials remain in all summary statistics. Failure reasons can include `timeout`, `endpoint_miss`, `insufficient_true_progress`, and `excessive_deviation`. The report shows evaluated successes alongside controller-stop rates so false completions remain visible.

## Outputs

- `summary.csv`: one row per trial, including seed, feedback, errors, timings, success, and failure reasons.
- `aggregate.csv`: statistics per route/scenario/feedback: means, sample standard deviations, observed min/max, and success rate. A single observation has reported standard deviation zero; it does not establish certainty.
- `paired.csv`: odometry-minus-ground-truth differences in tracking RMS, endpoint error, and duration, matched by route/scenario/seed.
- `comparison.png`: overview of mean per-run tracking/localization RMS and success fractions. Whiskers span observed min/max across selected routes/seeds; they are not confidence intervals.
- `report.md`: readable comparison with methods and limitations.
- `manifest.json`: success thresholds, seed list, source and simulator hashes, software environment, and all per-run configurations.
- Per-run `run.json`: configuration emitted by the simulator itself, including actual/assumed geometry and sensor parameters.
- Per-run `path.csv`, `trajectory.csv`, and `metrics.json`: replayable inputs and measurements.
- Per-run `tracking.png`: first listed seed for each route/scenario/feedback. `--animate` also generates a GIF for the first selected route/scenario/seed in each feedback mode.

The trajectory extends Stage 1 columns with `odometry_x/y/yaw`, wheel encoder increments (rad), wheel velocity noise (rad/s), and evaluation-only `true_progress` (m). State at timestamp t includes the encoder increment over the preceding interval. Commands and effective ground speeds at t apply to the following interval. Initial encoder/noise fields are zero. `left_actual` / `right_actual` are effective ground speeds, not shaft velocities during slip.

Position-localization RMS measures the supplied estimate against truth. `odometry_rms_m` separately evaluates the encoder estimate even when the controller uses ground truth. Heading-localization RMS wraps angular differences to avoid discontinuities at ±pi. Existing cross-track, final-pose, and controller timing metrics retain their Stage 1 definitions.

Per-run RMS covers that run's own duration. Paired runs can have different durations and stopping outcomes; inspect `paired.csv` and endpoint errors alongside RMS. The overview averages each run equally and mixes the selected route geometries. Use route-specific aggregates for more focused conclusions. Five seeds provide an initial comparison, not a broad reliability claim.

The RNG uses `std::mt19937` with `std::normal_distribution`. Replay is reproducible with the same binary/standard library except for measured timing; Gaussian sequences are not guaranteed identical across C++ library implementations. Manifests include the binary hash and source hashes to identify the implementation.

## Remaining work

Replicate these interfaces and experiments with an existing ROS 2/Gazebo robot. Then add IMU fusion and compare against wheel odometry with the same evaluation criteria and seed lists. Wheel odometry plus an IMU still lacks an absolute position reference; improvements must be measured rather than presumed.
