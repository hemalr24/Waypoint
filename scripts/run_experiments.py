#!/usr/bin/env python3
"""Run deterministic Stage 1 scenarios, report metrics, and render artifacts."""

import argparse
import csv
import json
import os
from pathlib import Path
import subprocess
import tempfile

os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "route-robot-mpl"))
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
ROUTES = ("line", "circle", "s_curve")
SCENARIOS = ("nominal", "offset", "stalled")


def metrics(data, path):
    """Distances use the true pose; heading error is wrapped to [-pi, pi]."""
    final = data[-1]
    heading = np.arctan2(path["y"][-1] - path["y"][-2], path["x"][-1] - path["x"][-2])
    angle = final["true_yaw"] - heading
    localization = np.hypot(data["true_x"] - data["estimated_x"],
                            data["true_y"] - data["estimated_y"])
    return {
        "completed": bool(final["complete"]),
        "timed_out": bool(final["timed_out"]),
        "duration_s": float(final["time"]),
        "cross_track_rms_m": float(np.sqrt(np.mean(data["cross_track_error"] ** 2))),
        "cross_track_max_m": float(np.max(data["cross_track_error"])),
        "final_position_error_m": float(np.hypot(final["true_x"] - path["x"][-1],
                                                  final["true_y"] - path["y"][-1])),
        "final_heading_error_rad": float(np.arctan2(np.sin(angle), np.cos(angle))),
        "localization_rms_m": float(np.sqrt(np.mean(localization ** 2))),
        "controller_mean_us": float(np.mean(data["controller_us"])),
        "controller_p99_us": float(np.percentile(data["controller_us"], 99)),
        "controller_max_us": float(np.max(data["controller_us"])),
    }


def render(data, path, destination, title, animate=False):
    fig = plt.figure(figsize=(11, 6), layout="constrained")
    grid = fig.add_gridspec(2, 2, width_ratios=[1.15, 1])
    map_ax = fig.add_subplot(grid[:, 0])
    error_ax = fig.add_subplot(grid[0, 1])
    speed_ax = fig.add_subplot(grid[1, 1])
    turn_ax = speed_ax.twinx()
    fig.suptitle(f"{title} | ground-truth feedback (Stage 1)")
    map_ax.plot(path["x"], path["y"], "--", color="0.55", label="Desired route")
    traveled, = map_ax.plot([], [], color="#0072B2", lw=2, label="True trajectory")
    estimated, = map_ax.plot([], [], color="#E69F00", ls=":", lw=2,
                             label="Estimated trajectory (= truth)")
    robot, = map_ax.plot([], [], "o", color="#0072B2", ms=8, label="Robot")
    target, = map_ax.plot([], [], "x", color="#CC79A7", ms=9, mew=2, label="Target")
    heading, = map_ax.plot([], [], color="#0072B2", lw=2)
    all_x = np.concatenate([path["x"], data["true_x"]])
    all_y = np.concatenate([path["y"], data["true_y"]])
    map_ax.set(xlim=(all_x.min() - 0.8, all_x.max() + 0.8),
               ylim=(all_y.min() - 0.8, all_y.max() + 0.8), xlabel="x [m]", ylabel="y [m]")
    map_ax.set_aspect("equal", adjustable="box")
    map_ax.legend(loc="upper left", fontsize=8)
    error_ax.plot(data["time"], data["cross_track_error"], color="#D55E00")
    error_ax.set(xlabel="Time [s]", ylabel="Cross-track error [m]", ylim=(0, None))
    speed_ax.plot(data["time"], data["linear"], color="#009E73", label="Forward")
    turn_ax.plot(data["time"], data["angular"], color="#CC79A7", label="Angular")
    speed_ax.set(xlabel="Time [s]", ylabel="Forward command [m/s]")
    turn_ax.set_ylabel("Angular command [rad/s]")
    speed_ax.legend(loc="upper left", fontsize=8)
    turn_ax.legend(loc="upper right", fontsize=8)
    cursors = [ax.axvline(0, color="0.4", lw=1) for ax in (error_ax, speed_ax)]
    for ax in (map_ax, error_ax, speed_ax):
        ax.grid(alpha=0.2)
    clock = map_ax.text(0.02, 0.02, "", transform=map_ax.transAxes, fontsize=9)

    def update(i):
        traveled.set_data(data["true_x"][:i+1], data["true_y"][:i+1])
        estimated.set_data(data["estimated_x"][:i+1], data["estimated_y"][:i+1])
        row = data[i]
        robot.set_data([row["true_x"]], [row["true_y"]])
        target.set_data([row["target_x"]], [row["target_y"]])
        heading.set_data([row["true_x"], row["true_x"] + 0.3*np.cos(row["true_yaw"])],
                         [row["true_y"], row["true_y"] + 0.3*np.sin(row["true_yaw"])])
        for cursor in cursors:
            cursor.set_xdata([row["time"], row["time"]])
        status = "complete" if row["complete"] else "timeout" if row["timed_out"] else "running"
        clock.set_text(f"t = {row['time']:.2f} s | {status}")
        return traveled, estimated, robot, target, heading, clock, *cursors

    update(len(data)-1)
    fig.savefig(destination / "tracking.png", dpi=140)
    if animate:
        # At most 120 frames keeps artifacts small; timestamp gives simulated time.
        frames = np.unique(np.linspace(0, len(data)-1, min(120, len(data))).astype(int))
        movie = FuncAnimation(fig, update, frames=frames, interval=80, blit=False)
        movie.save(destination / "tracking.gif", writer=PillowWriter(fps=12), dpi=80)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "results")
    parser.add_argument("--routes", nargs="+", choices=ROUTES, default=list(ROUTES))
    parser.add_argument("--scenarios", nargs="+", choices=SCENARIOS, default=list(SCENARIOS))
    parser.add_argument("--animate", action="store_true", help="Animate nominal runs for selected routes")
    args = parser.parse_args()
    executable = ROOT / "build" / "simulate"
    if not executable.exists():
        parser.error("simulator is not built; run make first")
    args.output.mkdir(parents=True, exist_ok=True)
    results = []
    for route in args.routes:
        for scenario in args.scenarios:
            destination = args.output / f"{route}_{scenario}"
            subprocess.run([str(executable), route, scenario, str(destination)], check=True)
            data = np.atleast_1d(np.genfromtxt(destination / "trajectory.csv", delimiter=",", names=True))
            path = np.genfromtxt(destination / "path.csv", delimiter=",", names=True)
            report = {"route": route, "scenario": scenario, "feedback": "ground_truth",
                      **metrics(data, path)}
            (destination / "metrics.json").write_text(json.dumps(report, indent=2) + "\n")
            render(data, path, destination, f"{route} / {scenario}", args.animate and scenario == "nominal")
            results.append(report)
    with (args.output / "summary.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(results[0]))
        writer.writeheader()
        writer.writerows(results)
    manifest = {
        "stage": 1, "feedback": "ground_truth", "simulation": "ideal differential-drive kinematics",
        "dt_s": 0.02, "timeout_s": 60, "random_seed": None,
        "randomness": "none; these scenarios are deterministic",
        "routes": args.routes, "scenarios": args.scenarios,
        "controller": {"lookahead_m": 0.5, "cruise_speed_m_s": 0.6,
                       "max_angular_speed_rad_s": 1.8, "track_width_m": 0.36,
                       "max_wheel_rim_speed_m_s": 0.9, "goal_tolerance_m": 0.06,
                       "approach_gain_per_s": 1.2},
        "scenarios_definition": {"nominal": "start at first point and first-segment heading",
                                 "offset": "nominal plus world y = 0.5 m and heading = 0.6 rad",
                                 "stalled": "nominal; both actual wheel speeds forced to zero at t >= 2 s"},
        "completion_rates": {scenario: sum(r["completed"] for r in results if r["scenario"] == scenario)
                             / sum(r["scenario"] == scenario for r in results)
                             for scenario in args.scenarios},
    }
    (args.output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    lines = ["# Stage 1 results", "", "Feedback: **simulator ground truth**. Deterministic kinematic runs.", "",
             "| Route | Scenario | Completed | RMS cross-track (m) | Max cross-track (m) | Final position (m) |",
             "|---|---|---|---:|---:|---:|"]
    for r in results:
        lines.append(f"| {r['route']} | {r['scenario']} | {r['completed']} | "
                     f"{r['cross_track_rms_m']:.4f} | {r['cross_track_max_m']:.4f} | {r['final_position_error_m']:.4f} |")
    lines += ["", "Completion rates across selected routes (not statistical reliability estimates):"]
    lines += [f"- {s}: {rate:.0%}" for s, rate in manifest["completion_rates"].items()]
    lines += ["", "A stalled robot may have near-zero cross-track error while making no progress. "
              "Always interpret tracking error alongside completion and endpoint error.", "",
              "Timing measures controller.update only; it excludes sensor processing, logging, middleware, "
              "and operating-system scheduling. It is not a real-time guarantee.", ""]
    (args.output / "report.md").write_text("\n".join(lines))
    print(f"Report: {args.output / 'report.md'}", flush=True)


if __name__ == "__main__":
    main()
