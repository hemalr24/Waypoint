"""Check recorded run semantics and independently recompute geometric error."""
import csv
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class ExperimentTests(unittest.TestCase):
    def run_scenario(self, scenario):
        with tempfile.TemporaryDirectory() as directory:
            subprocess.run([str(ROOT / "build/simulate"), "line", scenario, directory],
                           check=True, capture_output=True)
            with (Path(directory) / "trajectory.csv").open() as stream:
                return [{k: float(v) for k, v in row.items()} for row in csv.DictReader(stream)]

    def test_offset_recovers_and_log_is_consistent(self):
        rows = self.run_scenario("offset")
        self.assertEqual(rows[-1]["complete"], 1)
        self.assertEqual(rows[-1]["timed_out"], 0)
        for row in rows:
            x, y = row["true_x"], row["true_y"]
            nearest_x = max(0, min(6, x))
            self.assertAlmostEqual(row["cross_track_error"], ((x-nearest_x)**2+y*y)**0.5, places=9)
            self.assertEqual(row["true_x"], row["estimated_x"])
        self.assertLess(rows[-1]["cross_track_error"], 0.06)
        self.assertEqual(rows[-1]["linear"], 0)
        self.assertEqual(rows[-1]["angular"], 0)

    def test_stall_times_out_and_stops_commands(self):
        rows = self.run_scenario("stalled")
        self.assertEqual(rows[-1]["complete"], 0)
        self.assertEqual(rows[-1]["timed_out"], 1)
        self.assertEqual(rows[-1]["time"], 60)
        self.assertLess(rows[-1]["true_x"], 2)
        self.assertEqual(rows[-1]["linear"], 0)
        self.assertEqual(rows[-1]["angular"], 0)
        self.assertTrue(all(row["left_actual"] == row["right_actual"] == 0
                            for row in rows if row["time"] >= 2))

    def test_unknown_route_fails(self):
        with tempfile.TemporaryDirectory() as directory:
            result = subprocess.run([str(ROOT / "build/simulate"), "bogus", "nominal", directory],
                                    capture_output=True)
            self.assertNotEqual(result.returncode, 0)


if __name__ == "__main__":
    unittest.main()
