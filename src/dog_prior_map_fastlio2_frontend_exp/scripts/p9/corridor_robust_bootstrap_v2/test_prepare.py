"""Regress exact epoch carriers and inherited pre-run contracts."""
import json
import unittest
from prepare_v2 import ARCHIVE, OLD

class PreparationTests(unittest.TestCase):
    def test_exact_integer_ns_not_floating_epoch(self):
        c = json.loads((ARCHIVE / "bootstrap_v2_config.json").read_text())
        self.assertIs(type(c["bootstrap_start_ns"]), int)
        self.assertIs(type(c["bootstrap_end_ns"]), int)
        self.assertEqual(c["bootstrap_start_ns"], 1517157219088119030)
        self.assertEqual(c["bootstrap_end_ns"], 1517157229072630478)

    def test_inherited_quality_gates_unchanged(self):
        old = json.loads((OLD / "bootstrap_config.json").read_text())
        new = json.loads((ARCHIVE / "bootstrap_v2_config.json").read_text())
        for key in ("preintegration", "covariance", "frozen_formal_contract"):
            self.assertEqual(old[key], new[key])

    def test_single_scheme_conditional_support_is_fixed(self):
        c = json.loads((ARCHIVE / "bootstrap_v2_config.json").read_text())
        self.assertEqual(c["submap_frames"], 3)
        self.assertTrue(c["support_definition"]["no_result_dependent_recrop"])
        self.assertEqual(c["trajectory_gate"]["minimum_accepted_observations"], 90)
        self.assertEqual(c["trajectory_gate"]["maximum_consecutive_failures"], 2)

if __name__ == "__main__":
    unittest.main()
