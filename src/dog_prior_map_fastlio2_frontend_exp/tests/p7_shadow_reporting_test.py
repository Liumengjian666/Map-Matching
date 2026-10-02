#!/usr/bin/env python3
"""Posthoc reporting must not relabel numerical failures as missing map support."""
import importlib.util
from pathlib import Path
import unittest

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/p7_run_corridor01.py'
SPEC = importlib.util.spec_from_file_location('p7_corridor_wrapper', SCRIPT)
WRAPPER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(WRAPPER)


class ShadowReportingTest(unittest.TestCase):
    def test_failure_categories_remain_separate(self):
        def row(status, valid='0', computed='1', schur='NOT_COMPUTED'):
            return dict(uobs_computed=computed, uobs_valid=valid, map_support_status=status,
                        classification_valid=valid, weak_dimension='3', schur_status=schur)
        counts = WRAPPER.shadow_uobs_counts([
            row('MAP_SUPPORT_SUFFICIENT', '1', schur='VALID_DIAGNOSTIC'),
            row('MAP_SUPPORT_INSUFFICIENT'), row('NO_VALID_GEOMETRIC_CORRESPONDENCES'),
            row('NUMERICAL_FAILURE'), row('NOT_ASSESSED', computed='0'),
            row('MAP_SUPPORT_SUFFICIENT', '1', schur='SCHUR_DIAGNOSTIC_UNAVAILABLE')])
        self.assertEqual(counts['UOBS_COMPUTED'], 5)
        self.assertEqual(counts['UOBS_VALID'], 2)
        self.assertEqual(counts['UOBS_INVALID'], 3)
        self.assertEqual(counts['MAP_SUPPORT_INSUFFICIENT'], 1)
        self.assertEqual(counts['NO_VALID_GEOMETRIC_CORRESPONDENCES'], 1)
        self.assertEqual(counts['UOBS_NUMERICAL_FAILURE'], 1)
        self.assertEqual(counts['SCHUR_DIAGNOSTIC_UNAVAILABLE'], 1)
        self.assertEqual(counts['weak_dimension_histogram'], {3: 2})


if __name__ == '__main__':
    unittest.main()
