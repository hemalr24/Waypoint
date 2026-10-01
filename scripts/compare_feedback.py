#!/usr/bin/env python3
"""Paired feedback experiments: Stage 2 odometry or Stage 3 IMU fusion."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import platform
import subprocess

from run_experiments import ROOT, ROUTES, metrics, render, np, plt

SCENARIOS = ("nominal", "offset", "encoder_noise", "radius_mismatch", "wheel_slip", "combined", "stalled")
STOCHASTIC = {"encoder_noise", "combined"}
MODES = ("ground_truth", "odometry")
FUSION_MODES = (*MODES, "ekf")
FUSION_SCENARIOS = (*SCENARIOS, "gyro_bias", "symmetric_slip")
CRITERIA = {"final_position_tolerance_m": 0.15, "min_true_progress_fraction": 0.95,
            "max_cross_track_m": 1.0}


def evaluate(data, path):
    result = metrics(data, path)
    length = float(np.sum(np.hypot(np.diff(path["x"]), np.diff(path["y"]))))
    progress = float(data["true_progress"][-1] / length)
    odom_error = np.hypot(data["odometry_x"] - data["true_x"], data["odometry_y"] - data["true_y"])
    yaw_error = data["estimated_yaw"] - data["true_yaw"]
    yaw_error = np.arctan2(np.sin(yaw_error), np.cos(yaw_error))
    failures = []
    if not result["completed"]:
        failures.append("timeout" if result["timed_out"] else "no_controller_stop")
    if result["final_position_error_m"] > CRITERIA["final_position_tolerance_m"]:
        failures.append("endpoint_miss")
    if progress < CRITERIA["min_true_progress_fraction"]:
        failures.append("insufficient_true_progress")
    if result["cross_track_max_m"] > CRITERIA["max_cross_track_m"]:
        failures.append("excessive_deviation")
    result.update({"success": not failures, "failure_reason": ";".join(failures),
                   "true_progress_fraction": progress,
                   "odometry_rms_m": float(np.sqrt(np.mean(odom_error**2))),
                   "localization_heading_rms_rad": float(np.sqrt(np.mean(yaw_error**2))),
                   "final_localization_error_m": float(np.hypot(data["estimated_x"][-1]-data["true_x"][-1],
                                                               data["estimated_y"][-1]-data["true_y"][-1]))})
    if "ekf_x" in data.dtype.names:
        ekf_position = np.hypot(data["ekf_x"]-data["true_x"], data["ekf_y"]-data["true_y"])
        ekf_yaw = data["ekf_yaw"]-data["true_yaw"]
        odom_yaw = data["odometry_yaw"]-data["true_yaw"]
        result.update({"ekf_rms_m": float(np.sqrt(np.mean(ekf_position**2))),
                       "ekf_heading_rms_rad": float(np.sqrt(np.mean(np.arctan2(np.sin(ekf_yaw), np.cos(ekf_yaw))**2))),
                       "odometry_heading_rms_rad": float(np.sqrt(np.mean(np.arctan2(np.sin(odom_yaw), np.cos(odom_yaw))**2))),
                       "ekf_p99_us": float(np.percentile(data["ekf_us"][1:], 99)) if len(data)>1 else 0.0})
    return result


def write_csv(path, rows):
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def aggregate(rows, keys):
    groups = {}
    for row in rows:
        groups.setdefault(tuple(row[key] for key in keys), []).append(row)
    summaries = []
    for group, samples in groups.items():
        summary = dict(zip(keys, group))
        summary.update({"trials": len(samples), "successes": sum(r["success"] for r in samples),
                        "success_rate": float(np.mean([r["success"] for r in samples])),
                        "controller_stop_rate": float(np.mean([r["completed"] for r in samples]))})
        for metric in ("cross_track_rms_m", "cross_track_max_m", "final_position_error_m", "localization_rms_m",
                       "localization_heading_rms_rad", "duration_s", "controller_p99_us"):
            values = np.array([r[metric] for r in samples])
            summary.update({f"{metric}_mean": float(values.mean()),
                            f"{metric}_min": float(values.min()), f"{metric}_max": float(values.max()),
                            f"{metric}_std": float(values.std(ddof=1)) if len(values) > 1 else 0.0})
        summaries.append(summary)
    return summaries


def comparison_plot(summaries, scenarios, output, modes=MODES, stage=2):
    fig, axes = plt.subplots(1, 3, figsize=(17, 5), layout="constrained")
    x = np.arange(len(scenarios))
    width = 0.8/len(modes)
    shifts = (np.arange(len(modes))-(len(modes)-1)/2)*width
    for mode, shift, color in zip(modes, shifts, ("#0072B2", "#D55E00", "#009E73")):
        rows = [next(r for r in summaries if r["scenario"] == s and r["feedback"] == mode) for s in scenarios]
        for ax, metric, title in zip(axes[:2], ("cross_track_rms_m", "localization_rms_m"),
                                     ("True tracking error", "Controller localization error")):
            means = np.array([r[f"{metric}_mean"] for r in rows])
            low = means - np.array([r[f"{metric}_min"] for r in rows])
            high = np.array([r[f"{metric}_max"] for r in rows]) - means
            ax.errorbar(x+shift, means, yerr=np.array([low, high]), fmt="o", capsize=3, color=color, label=mode)
            ax.set(title=title, ylabel="Mean per-run RMS [m]")
        axes[2].bar(x+shift, [r["success_rate"] for r in rows], width=width, color=color, label=mode)
    # Set lower bounds only after both modes have contributed to autoscaling.
    for ax in axes[:2]:
        ax.autoscale(enable=True, axis="y")
        ax.set_ylim(bottom=0)
    axes[2].set(title="Evaluated success", ylabel="Fraction of runs", ylim=(0, 1.08))
    for ax in axes:
        ax.set_xticks(x, [s.replace("_", "\n") for s in scenarios], fontsize=8)
        ax.grid(axis="y", alpha=0.2)
        ax.legend(fontsize=8)
    fig.suptitle(f"Stage {stage}: paired feedback comparison\nMeans across selected routes/seeds; whiskers show observed min–max, not confidence intervals")
    fig.savefig(output / "comparison.png", dpi=140)
    plt.close(fig)


def uint32(value):
    try:
        number = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("seed must be an integer") from exc
    if not 0 <= number <= 2**32-1:
        raise argparse.ArgumentTypeError("seed must be in [0, 4294967295]")
    return number


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", type=int, choices=(2, 3), default=2)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--routes", nargs="+", choices=ROUTES, default=list(ROUTES))
    parser.add_argument("--scenarios", nargs="+", choices=FUSION_SCENARIOS)
    parser.add_argument("--seeds", nargs="+", type=uint32, default=[11, 22, 33, 44, 55])
    parser.add_argument("--no-plots", action="store_true", help="Skip per-run and comparison plots")
    parser.add_argument("--animate", action="store_true", help="Animate first route/scenario/seed in all selected modes")
    args = parser.parse_args()
    modes = MODES if args.stage == 2 else FUSION_MODES
    scenarios = SCENARIOS if args.stage == 2 else FUSION_SCENARIOS
    args.scenarios = args.scenarios or list(scenarios)
    if any(s not in scenarios for s in args.scenarios):
        parser.error("gyro_bias and symmetric_slip require --stage 3")
    args.output = args.output or ROOT / "results" / f"stage{args.stage}"
    stochastic = STOCHASTIC if args.stage == 2 else set(args.scenarios)
    for field in ("routes", "scenarios", "seeds"):
        values = getattr(args, field)
        if len(set(values)) != len(values):
            parser.error(f"duplicate {field} would double-count trials")
    if args.animate and args.no_plots:
        parser.error("--animate requires plots")
    executable = ROOT / "build" / "simulate"
    if not executable.exists():
        parser.error("run make first")
    args.output.mkdir(parents=True, exist_ok=True)
    results, runs = [], []
    for route in args.routes:
        for scenario in args.scenarios:
            # Repeating identical deterministic runs adds no statistical evidence.
            seeds = args.seeds if scenario in stochastic else args.seeds[:1]
            for seed in seeds:
                for feedback in modes:
                    directory = args.output / f"{route}_{scenario}_{feedback}_seed{seed}"
                    subprocess.run([str(executable), route, scenario, str(directory), feedback, str(seed)],
                                   check=True, capture_output=True, text=True)
                    data = np.atleast_1d(np.genfromtxt(directory / "trajectory.csv", delimiter=",", names=True))
                    path = np.genfromtxt(directory / "path.csv", delimiter=",", names=True)
                    row = {"route": route, "scenario": scenario, "feedback": feedback, "seed": seed, **evaluate(data, path)}
                    (directory / "metrics.json").write_text(json.dumps(row, indent=2) + "\n")
                    results.append(row)
                    runs.append({"directory": directory.name, "configuration": json.loads((directory / "run.json").read_text())})
                    if not args.no_plots and seed == seeds[0]:
                        animate = args.animate and route == args.routes[0] and scenario == args.scenarios[0]
                        render(data, path, directory, f"{route} / {scenario} / seed {seed}", animate, feedback)
            print(f"{route}/{scenario}: {len(seeds)} paired trial(s)", flush=True)
    write_csv(args.output / "summary.csv", results)
    summaries = aggregate(results, ("route", "scenario", "feedback"))
    write_csv(args.output / "aggregate.csv", summaries)
    overview = aggregate(results, ("scenario", "feedback"))
    paired = []
    index = {(r["route"], r["scenario"], r["seed"], r["feedback"]): r for r in results}
    for row in results:
        if row["feedback"] == "ground_truth":
            continue
        baselines = ("ground_truth", "odometry") if row["feedback"] == "ekf" else ("ground_truth",)
        for baseline_mode in baselines:
            baseline = index[(row["route"], row["scenario"], row["seed"], baseline_mode)]
            paired.append({"route": row["route"], "scenario": row["scenario"], "seed": row["seed"],
                           "baseline_feedback": baseline_mode, "feedback": row["feedback"],
                           "baseline_success": baseline["success"], "success": row["success"],
                           "delta_cross_track_rms_m": row["cross_track_rms_m"]-baseline["cross_track_rms_m"],
                           "delta_final_position_error_m": row["final_position_error_m"]-baseline["final_position_error_m"],
                           "delta_localization_rms_m": row["localization_rms_m"]-baseline["localization_rms_m"],
                           "delta_duration_s": row["duration_s"]-baseline["duration_s"]})
    write_csv(args.output / "paired.csv", paired)
    if args.stage == 3:
        write_csv(args.output / "same_run_estimators.csv", [
            {"route": r["route"], "scenario": r["scenario"], "trajectory_feedback": r["feedback"], "seed": r["seed"],
             "odometry_rms_m": r["odometry_rms_m"], "ekf_rms_m": r["ekf_rms_m"],
             "delta_rms_m": r["ekf_rms_m"]-r["odometry_rms_m"],
             "odometry_heading_rms_rad": r["odometry_heading_rms_rad"], "ekf_heading_rms_rad": r["ekf_heading_rms_rad"],
             "ekf_p99_us": r["ekf_p99_us"]} for r in results])
    source_paths = [ROOT / "Makefile", *sorted((ROOT / "src").glob("*.cpp")),
                    *sorted((ROOT / "include/route_robot").glob("*.hpp")),
                    *sorted((ROOT / "scripts").glob("*.py"))]
    manifest = {"stage": args.stage, "simulation": "kinematic harness; no contact dynamics", "feedback_modes": modes,
                "seeds": args.seeds, "stochastic_scenarios": sorted(stochastic), "criteria": CRITERIA,
                "deterministic_repetition": "first seed only in Stage 2; all seeds in Stage 3 because gyro noise is always active", "routes": args.routes, "scenarios": args.scenarios,
                "python": platform.python_version(), "numpy": np.__version__, "platform": platform.platform(),
                "simulator_sha256": hashlib.sha256(executable.read_bytes()).hexdigest(),
                "source_sha256": {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in source_paths},
                "rng": "std::mt19937 + std::normal_distribution; repeatable within the same C++ standard library",
                "runs": runs}
    (args.output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    if not args.no_plots:
        comparison_plot(overview, args.scenarios, args.output, modes, args.stage)
    lines = [f"# Stage {args.stage}: " + ("wheel-odometry feedback" if args.stage == 2 else "wheel odometry and IMU fusion"), "",
             "Paired runs share route, initial pose, controller settings, disturbance schedule, and noise seed.",
             "After known-pose initialization, estimators receive sensor measurements only. Truth supplies sensor simulation and evaluation.", "",
             f"Evaluated success requires a controller stop, final true position within {CRITERIA['final_position_tolerance_m']} m, "
             f"at least {CRITERIA['min_true_progress_fraction']:.0%} true route progress, and maximum cross-track error "
             f"no greater than {CRITERIA['max_cross_track_m']} m. Heading is measured but not a pass criterion.", "",
             "| Scenario | Feedback | Successes / runs | Controller stops | Mean RMS tracking (m) | Mean RMS localization (m) | Mean endpoint error (m) |",
             "|---|---|---:|---:|---:|---:|---:|"]
    for r in overview:
        lines.append(f"| {r['scenario']} | {r['feedback']} | {r['successes']}/{r['trials']} | "
                     f"{r['controller_stop_rate']:.0%} | {r['cross_track_rms_m_mean']:.4f} | "
                     f"{r['localization_rms_m_mean']:.4f} | {r['final_position_error_m_mean']:.4f} |")
    lines += ["", "## Interpretation", "",
              "Controller stops are based on the supplied estimate. A robot can stop near its estimated endpoint while its true position misses the goal. "
              "Successes use separate truth-based evaluation; failed runs remain in every aggregate.", "",
              "Encoder noise perturbs measured wheel velocity. Radius mismatch changes physical motion relative to assumed geometry. "
              "Slip reduces ground displacement while encoder shafts keep rotating. The stalled fixture stops the shafts themselves.", "",
              "Overview values are equally weighted per-run means across selected routes and seeds. Stage 2 repeats encoder-noisy scenarios only; "
              "Stage 3 uses every seed for every mode/scenario because gyro noise is always active. Results describe this finite test set, not real-world reliability. "
              "Per-run errors cover each run's own duration, which may differ between feedback modes; paired.csv records duration differences.", "",
              "Plots show the first listed seed without selecting for outcome. comparison.png shows observed ranges, not confidence intervals. "
              "When --no-plots is used, no new plots are generated.", "",
              "See aggregate.csv for route-specific means, sample standard deviations, and observed ranges; summary.csv and per-run metrics.json "
              "retain individual outcomes and failure reasons. run.json records simulator parameters; manifest.json records software/source hashes.", "",
              "The kinematic harness omits rigid-body contact dynamics, encoder bias/quantization, and actuator lag. Initial pose is known. "
              "Controller timing includes only controller.update; EKF update timing is logged separately. Neither is a real-time guarantee. ROS 2/Gazebo integration remains pending.", ""]
    if args.stage == 3:
        lines += ["## Fusion interpretation", "",
                  "The EKF fuses wheel forward/turn rates with gyro turn rate using a fixed five-state planar model. "
                  "It receives no absolute heading or position. Gyro bias is unmodeled; symmetric slip corrupts forward displacement. "
                  "Neither failure mode is guaranteed to improve with fusion.", "",
                  "same_run_estimators.csv compares passive odometry and EKF estimates using identical sensor histories and duration on each trajectory. "
                  "paired.csv additionally compares EKF-controlled runs with odometry-controlled runs; those physical trajectories may differ.", "",
                  "All modes receive paired encoder and gyro noise streams. Baseline modes ignore gyro data for control. "
                  "The fixed covariance tuning is recorded in run.json, including its mismatch with biased/slipping sensor conditions. "
                  "Covariance is the filter's modeled uncertainty, not a guarantee of actual accuracy.", ""]
    (args.output / "report.md").write_text("\n".join(lines))
    print(f"Report: {args.output / 'report.md'}", flush=True)


if __name__ == "__main__":
    main()
