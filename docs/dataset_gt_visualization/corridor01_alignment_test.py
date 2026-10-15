#!/usr/bin/env python3
"""Pure calibration-guard tests. No bag, GT read or registration is executed."""
import importlib.util
from pathlib import Path
import unittest
from types import SimpleNamespace

import numpy as np

SPEC = importlib.util.spec_from_file_location(
    'alignment', Path(__file__).with_name('align_corridor01_map.py'))
ALIGN = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ALIGN)


class AlignmentGuardTest(unittest.TestCase):
    def test_nearest_sensor_header_not_bag_record(self):
        def timestamp(ns):
            return SimpleNamespace(to_nsec=lambda: ns)

        def row(header_ns, record_ns):
            return ('points', SimpleNamespace(header=SimpleNamespace(
                stamp=timestamp(header_ns))), timestamp(record_ns))

        rows = [row(100, 1000000), row(149, 2000000), row(151, 3000000)]
        selected = ALIGN.nearest_headers(rows, {0: 150, 1: 101})
        # Exact tie chooses the earlier header, not the record stamp.
        self.assertEqual(selected[0][1].header.stamp.to_nsec(), 149)
        self.assertEqual(selected[0][2], 2000000)
        self.assertEqual(selected[1][1].header.stamp.to_nsec(), 100)
        with self.assertRaises(ValueError):
            ALIGN.nearest_headers([rows[1], rows[0]], {0: 150})
        with self.assertRaises(ValueError):
            ALIGN.nearest_headers([], {0: 150})

    def test_proper_se3_only(self):
        self.assertTrue(ALIGN.rigid(np.eye(4)))
        for diagonal in ([1., -1., 1., 1.], [2., 2., 2., 1.]):
            self.assertFalse(ALIGN.rigid(np.diag(diagonal)))
        bad = np.eye(4)
        bad[3, 0] = .1
        self.assertFalse(ALIGN.rigid(bad))
        bad = np.eye(4)
        bad[0, 3] = np.nan
        self.assertFalse(ALIGN.rigid(bad))

    def test_noncommuting_transform_direction(self):
        transform = np.array([[0., -1., 0., 3.], [1., 0., 0., 4.],
                              [0., 0., 1., 5.], [0., 0., 0., 1.]])
        point = np.array([[1., 2., 3.]])
        np.testing.assert_allclose(ALIGN.moved(point, transform), [[1., 5., 8.]])
        np.testing.assert_allclose(
            ALIGN.moved(ALIGN.moved(point, transform), np.linalg.inv(transform)), point)

    def test_no_acceptance_on_median_alone_or_nonfinite_statistics(self):
        good = dict(median_m=1., p90_m=3., coverage_0_5m=.5, coverage_2m=.8)
        self.assertTrue(ALIGN.acceptable(good))
        for key, value in (('median_m', 1.001), ('p90_m', 3.001),
                           ('coverage_0_5m', .499), ('coverage_2m', .799)):
            self.assertFalse(ALIGN.acceptable(dict(good, **{key: value})))
        for key in good:
            self.assertFalse(ALIGN.acceptable(dict(good, **{key: np.nan})))


if __name__ == '__main__':
    unittest.main()
