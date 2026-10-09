import importlib.util
import math
import unittest

import numpy as np

SCRIPT = __file__.replace("test_contract.py", "freeze_predictors.py")
SPEC = importlib.util.spec_from_file_location("freeze_predictors", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
EVALUATOR_SCRIPT = __file__.replace("test_contract.py", "evaluate.py")
EVALUATOR_SPEC = importlib.util.spec_from_file_location("paired_evaluator", EVALUATOR_SCRIPT)
EVALUATOR = importlib.util.module_from_spec(EVALUATOR_SPEC)
EVALUATOR_SPEC.loader.exec_module(EVALUATOR)


def row(tx, start, end):
    return {"method": "NOMINAL", "transaction_id": str(tx),
            "scan_start_ns": str(start), "scan_end_ns": str(end)}


class FixedWindowContract(unittest.TestCase):
    def test_boundary_scan_is_excluded_and_full_scans_selected(self):
        origin = MODULE.EVAL_ORIGIN_NS
        rows = [row(51, origin - 100, origin + 42_000_000),
                row(52, origin + 42_698_055, origin + 143_537_266),
                row(53, origin + 143_554_936, origin + 244_397_385),
                row(398, origin + 34_938_123_000, origin + 35_039_000_000)]
        selected = MODULE.selected_rows(rows)
        self.assertEqual([int(x["transaction_id"]) for x in selected], [52, 53])

    def test_no_method_or_score_based_selection(self):
        origin = MODULE.EVAL_ORIGIN_NS
        rows = [row(52, origin + 1, origin + 2),
                {**row(53, origin + 2, origin + 3), "method": "COUPLED"}]
        self.assertEqual([int(x["transaction_id"]) for x in MODULE.selected_rows(rows)], [52])

    def test_fixed_prefix_rigid_alignment_recovery(self):
        source = np.array([[0., 0., 0.], [1., 0., 0.], [0., 2., 0.], [0., 0., 3.]])
        angle = 0.37
        rotation = np.array([[math.cos(angle), -math.sin(angle), 0.],
                             [math.sin(angle), math.cos(angle), 0.], [0., 0., 1.]])
        translation = np.array([3., -2., .5])
        target = (rotation @ source.T).T + translation
        fitted_rotation, fitted_translation = EVALUATOR.kabsch(source, target)
        self.assertLess(np.linalg.norm(fitted_rotation-rotation), 1e-12)
        self.assertLess(np.linalg.norm(fitted_translation-translation), 1e-12)
        self.assertAlmostEqual(np.linalg.det(fitted_rotation), 1.0, places=12)

    def test_persistent_failure_requires_five_seconds(self):
        times = [i * .5 for i in range(13)]
        effective = [i == 0 for i in range(13)]
        result = EVALUATOR.persistent_failure(times, effective)
        self.assertEqual(result["first_strict_failure_s"], .5)
        self.assertEqual(result["first_persistent_failure_s"], .5)
        self.assertEqual(result["longest_consecutive_failure_frames"], 12)


if __name__ == "__main__":
    unittest.main()
