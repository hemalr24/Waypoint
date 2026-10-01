"""Integration checks for estimator isolation, disturbances, and paired reports."""
import csv
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from compare_feedback import comparison_plot, evaluate, np


class OdometryExperiments(unittest.TestCase):
    def run_case(self, scenario, feedback="odometry", seed=11):
        with tempfile.TemporaryDirectory() as directory:
            subprocess.run([str(ROOT / "build/simulate"), "line", scenario, directory, feedback, str(seed)],
                           check=True, capture_output=True)
            directory = Path(directory)
            return (np.genfromtxt(directory / "trajectory.csv", delimiter=",", names=True),
                    np.genfromtxt(directory / "path.csv", delimiter=",", names=True),
                    json.loads((directory / "run.json").read_text()))

    def test_ideal_odometry_matches_truth(self):
        data, path, _ = self.run_case("nominal")
        for dimension in ("x", "y", "yaw"):
            np.testing.assert_allclose(data[f"estimated_{dimension}"], data[f"true_{dimension}"], atol=1e-10)
        self.assertTrue(evaluate(data, path)["success"])

    def test_slip_does_not_leak_truth_into_odometry(self):
        slipped, path, _ = self.run_case("wheel_slip")
        nominal, _, _ = self.run_case("nominal")
        # Same shaft rotations and initial pose => identical odometry/control,
        # even though slip physically displaces the robot from its intended route.
        self.assertEqual(len(slipped), len(nominal))
        for field in ("estimated_x", "estimated_y", "estimated_yaw", "linear", "angular"):
            np.testing.assert_array_equal(slipped[field], nominal[field])
        self.assertGreater(np.max(np.abs(slipped["true_y"]-nominal["true_y"])), 1.0)
        result = evaluate(slipped, path)
        self.assertTrue(result["completed"])
        self.assertFalse(result["success"])
        self.assertIn("endpoint_miss", result["failure_reason"])
        # Encoder increment on row i spans the preceding interval [i-1, i].
        i = np.flatnonzero(slipped["time"] >= 3)[0]
        self.assertAlmostEqual(slipped["left_encoder_delta"][i]*0.1/0.02,
                               slipped["left_command"][i-1], places=9)
        self.assertAlmostEqual(slipped["left_actual"][i], slipped["left_command"][i]*0.65, places=9)

    def test_seed_replay_and_paired_noise(self):
        a, _, _ = self.run_case("encoder_noise", seed=22)
        b, _, _ = self.run_case("encoder_noise", seed=22)
        c, _, _ = self.run_case("encoder_noise", "ground_truth", seed=22)
        d, _, _ = self.run_case("encoder_noise", seed=33)
        for field in a.dtype.names:
            if field != "controller_us":
                np.testing.assert_array_equal(a[field], b[field])
        common = min(len(a), len(c))
        np.testing.assert_array_equal(a["left_encoder_noise"][:common], c["left_encoder_noise"][:common])
        np.testing.assert_array_equal(a["right_encoder_noise"][:common], c["right_encoder_noise"][:common])
        self.assertNotEqual(a["left_encoder_noise"][1], d["left_encoder_noise"][1])
        np.testing.assert_array_equal(c["estimated_x"], c["true_x"])
        self.assertGreater(np.max(np.abs(c["odometry_y"]-c["true_y"])), 0.001)

    def test_radius_mismatch_and_configuration(self):
        data, path, config = self.run_case("radius_mismatch")
        self.assertEqual(config["actual_left_radius_m"], 0.098)
        self.assertEqual(config["actual_right_radius_m"], 0.102)
        self.assertEqual(config["encoder_velocity_noise_std_rad_s"], 0)
        self.assertGreater(evaluate(data, path)["localization_rms_m"], 0.1)

    def test_stalled_encoders_stop_and_timeout(self):
        data, path, _ = self.run_case("stalled")
        self.assertTrue(data["timed_out"][-1])
        self.assertFalse(evaluate(data, path)["success"])
        np.testing.assert_array_equal(data["left_encoder_delta"][data["time"] > 2], 0)
        self.assertEqual(data["linear"][-1], 0)

    def test_closed_path_start_is_not_evaluated_as_success(self):
        data, _, _ = self.run_case("nominal")
        fake = data[-1:].copy()
        fake["true_x"] = 0
        fake["true_y"] = 0
        fake["true_progress"] = 0
        circle = np.array([(0., 0.), (1., 0.), (1., 1.), (0., 0.)], dtype=[("x", float), ("y", float)])
        self.assertFalse(evaluate(fake, circle)["success"])
        self.assertIn("insufficient_true_progress", evaluate(fake, circle)["failure_reason"])

    def test_runner_keeps_failures_and_pairs_without_duplicate_deterministic_runs(self):
        with tempfile.TemporaryDirectory() as directory:
            subprocess.run([sys.executable, str(ROOT / "scripts/compare_feedback.py"), "--output", directory,
                            "--routes", "line", "--scenarios", "wheel_slip", "encoder_noise",
                            "--seeds", "11", "22", "--no-plots"], check=True, capture_output=True)
            destination = Path(directory)
            with (destination / "summary.csv").open() as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 6)
            slip = next(r for r in rows if r["feedback"] == "odometry" and r["scenario"] == "wheel_slip")
            self.assertEqual(slip["success"], "False")
            with (destination / "paired.csv").open() as stream:
                self.assertEqual(len(list(csv.DictReader(stream))), 3)
            manifest = json.loads((destination / "manifest.json").read_text())
            self.assertEqual(len(manifest["runs"]), 6)
            self.assertIn("criteria", manifest)
            self.assertTrue((destination / "report.md").exists())

    def test_bad_seed_rejected(self):
        for seed in ("-1", "4294967296", "12x"):
            with tempfile.TemporaryDirectory() as directory:
                result = subprocess.run([str(ROOT / "build/simulate"), "line", "nominal", directory,
                                         "odometry", seed], capture_output=True)
                self.assertNotEqual(result.returncode, 0)

    def test_comparison_plot_includes_larger_second_mode_errors(self):
        summaries = []
        for mode, mean, high in (("ground_truth", 0.01, 0.02), ("odometry", 3.0, 5.0)):
            row = {"scenario": "wheel_slip", "feedback": mode, "success_rate": 0.5}
            for metric in ("cross_track_rms_m", "localization_rms_m"):
                row.update({f"{metric}_mean": mean, f"{metric}_min": 0, f"{metric}_max": high})
            summaries.append(row)
        with patch("matplotlib.figure.Figure.savefig", autospec=True) as save:
            comparison_plot(summaries, ["wheel_slip"], Path("unused"))
            figure = save.call_args.args[0]
            for axes in figure.axes[:2]:
                self.assertGreaterEqual(axes.get_ylim()[1], 5.0)


if __name__ == "__main__":
    unittest.main()
