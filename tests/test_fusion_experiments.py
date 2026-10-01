"""Sensor timing, estimator isolation, replay, and Stage 3 reporting checks."""
import csv
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from compare_feedback import np, evaluate


class FusionExperiments(unittest.TestCase):
    def run_case(self, scenario, feedback="ekf", seed=11):
        with tempfile.TemporaryDirectory() as directory:
            subprocess.run([str(ROOT / "build/simulate"), "line", scenario, directory, feedback, str(seed)],
                           check=True, capture_output=True)
            folder = Path(directory)
            return (np.genfromtxt(folder / "trajectory.csv", delimiter=",", names=True),
                    np.genfromtxt(folder / "path.csv", delimiter=",", names=True),
                    json.loads((folder / "run.json").read_text()))

    def test_gyro_observes_physical_turn_rate_and_previous_interval(self):
        data, _, config = self.run_case("combined")
        expected = (data["right_actual"][:-1]-data["left_actual"][:-1])/config["actual_track_width_m"]
        np.testing.assert_allclose(data["gyro_rate"][1:]-data["gyro_noise"][1:], expected, atol=1e-9)
        self.assertEqual(data["gyro_rate"][0], 0)
        self.assertEqual(config["gyro_noise_std_rad_s"], 0.02)
        self.assertEqual(config["ekf"]["wheel_turn_std_rad_s"], 0.25)

    def test_filter_has_no_truth_position_leak(self):
        normal, _, _ = self.run_case("nominal", "odometry")
        slipping, _, _ = self.run_case("symmetric_slip", "odometry")
        # Straight, equal-wheel slip preserves yaw and encoder readings but changes x.
        for field in ("left_encoder_delta", "right_encoder_delta", "gyro_rate", "ekf_x", "ekf_y", "ekf_yaw"):
            np.testing.assert_array_equal(normal[field], slipping[field])
        self.assertGreater(abs(normal["true_x"][-1]-slipping["true_x"][-1]), 1)

    def test_three_modes_share_noise_and_fusion_replays(self):
        fused, _, _ = self.run_case("encoder_noise")
        repeated, _, _ = self.run_case("encoder_noise")
        for field in fused.dtype.names:
            if field not in ("controller_us", "ekf_us"):
                np.testing.assert_array_equal(fused[field], repeated[field])
        for mode in ("ground_truth", "odometry"):
            other, _, _ = self.run_case("encoder_noise", mode)
            n = min(len(fused), len(other))
            for field in ("left_encoder_noise", "right_encoder_noise", "gyro_noise"):
                np.testing.assert_array_equal(fused[field][:n], other[field][:n])
        for dimension in ("x", "y", "yaw"):
            np.testing.assert_array_equal(fused[f"estimated_{dimension}"], fused[f"ekf_{dimension}"])
        for dimension in ("x", "y", "yaw", "speed", "turn"):
            self.assertTrue(np.all(np.isfinite(fused[f"ekf_var_{dimension}"])))
            self.assertTrue(np.all(fused[f"ekf_var_{dimension}"] > 0))

    def test_heading_help_and_unobservable_translation_failure(self):
        data, _, _ = self.run_case("radius_mismatch", "ground_truth")
        odom_angle = np.arctan2(np.sin(data["odometry_yaw"]-data["true_yaw"]),
                                np.cos(data["odometry_yaw"]-data["true_yaw"]))
        ekf_angle = np.arctan2(np.sin(data["ekf_yaw"]-data["true_yaw"]),
                               np.cos(data["ekf_yaw"]-data["true_yaw"]))
        self.assertLess(np.mean(ekf_angle**2), np.mean(odom_angle**2)*0.1)
        slipped, path, _ = self.run_case("symmetric_slip")
        result = evaluate(slipped, path)
        self.assertFalse(result["success"])
        self.assertGreater(result["final_position_error_m"], 0.9)

    def test_gyro_bias_can_make_fusion_worse(self):
        fused, path, _ = self.run_case("gyro_bias")
        odom, _, _ = self.run_case("gyro_bias", "odometry")
        self.assertFalse(evaluate(fused, path)["success"])
        self.assertTrue(evaluate(odom, path)["success"])

    def test_stage3_reports_all_seeded_modes_and_same_history_metrics(self):
        with tempfile.TemporaryDirectory() as directory:
            subprocess.run([sys.executable, str(ROOT / "scripts/compare_feedback.py"), "--stage", "3",
                            "--routes", "line", "--scenarios", "nominal", "gyro_bias", "--seeds", "11", "22",
                            "--no-plots", "--output", directory], check=True, capture_output=True)
            folder = Path(directory)
            for filename, expected in (("summary.csv", 12), ("paired.csv", 12), ("same_run_estimators.csv", 12)):
                with (folder / filename).open() as stream:
                    self.assertEqual(len(list(csv.DictReader(stream))), expected)
            manifest = json.loads((folder / "manifest.json").read_text())
            self.assertEqual(manifest["stage"], 3)
            self.assertEqual(manifest["feedback_modes"], ["ground_truth", "odometry", "ekf"])
            self.assertEqual(len(manifest["runs"]), 12)


if __name__ == "__main__":
    unittest.main()
