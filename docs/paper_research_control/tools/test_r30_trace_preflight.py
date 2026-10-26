#!/usr/bin/env python3
"""Standard-library tests; no ROS, GT or sensor data required."""
import csv
import importlib.util
from pathlib import Path
import tempfile
import unittest

SCRIPT = Path(__file__).with_name("r30_trace_preflight.py")
spec = importlib.util.spec_from_file_location("r30_trace_preflight", SCRIPT)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

FIELDS = ("raw_ordinal", "scan_stamp_ns", "state_stamp_ns", "px", "py", "pz", "qx", "qy", "qz", "qw", "map_published", "ekf_update_applied", "reference_valid", "reference_high_error", "reason", "translation_error_m", "rotation_error_deg", "extra_calls")


def sample(raw, **updates):
    values = dict(raw_ordinal=raw, scan_stamp_ns=str(raw*1000000), state_stamp_ns=str(raw*1000000),
                  px="0", py="0", pz="0", qx="0", qy="0", qz="0", qw="1",
                  map_published="False", ekf_update_applied="False", reference_valid="True",
                  reference_high_error="False", reason="NO_ACCEPTABLE_MAP_MEASUREMENT",
                  translation_error_m="0.1", rotation_error_deg="1", extra_calls="0")
    values.update(updates)
    return values


class FirstDivergenceTest(unittest.TestCase):
    def test_full_accounting_and_b_c_same_state_different_search_calls(self):
        with tempfile.TemporaryDirectory() as root:
            archive = Path(root)
            directory = archive / "FULL_SEQUENCE_TRAJECTORIES"
            directory.mkdir()
            base = [sample(5), sample(6, reference_high_error="True", map_published="True", ekf_update_applied="True"), sample(7, reference_valid="False")]
            for method in "ABCD":
                rows = [dict(row) for row in base]
                if method == "C":
                    for row in rows: row["extra_calls"] = "2"
                if method == "A": rows[1]["px"] = "3"
                if method == "D": rows[2]["px"] = "5"
                with (directory / ("EASY_C0_"+method+".csv")).open("w", newline="", encoding="utf8") as file:
                    w = csv.DictWriter(file, FIELDS)
                    w.writeheader();w.writerows(rows)
            result = module.audit(archive)
            self.assertIsNone(result["first_state_divergence"]["B_vs_C"])
            self.assertEqual(result["first_state_divergence"]["A_vs_B"]["raw_ordinal"], 6)
            self.assertEqual(result["summary"]["C"]["extra_ndt_calls"], 6)
            self.assertEqual(result["summary"]["C"]["unavailable_reference_rows"], [7])
            self.assertEqual(result["summary"]["C"]["first_reference_high_error_raw"], 6)
            self.assertEqual(result["summary"]["C"]["actual_ekf_updates"], 1)

    def test_gap_in_scan_ordinals_is_rejected(self):
        with tempfile.TemporaryDirectory() as root:
            path = Path(root) / "bad.csv"
            with path.open("w", newline="", encoding="utf8") as file:
                w = csv.DictWriter(file, FIELDS)
                w.writeheader();w.writerows([sample(5),sample(7)])
            with self.assertRaises(ValueError):
                module.read_rows(path)


if __name__ == "__main__":
    unittest.main()
