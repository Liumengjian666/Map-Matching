import unittest
import numpy as np
from basin_math import effective, perturbations, uniform_rows


class SchurTests(unittest.TestCase):
    def test_profile_energy_and_independent_solve(self):
        B = np.diag([.5, 1., 2.])
        H = np.block([[np.diag([1., 3., 6.]), B], [B, np.eye(3) * 4]])
        r = effective(H)
        np.testing.assert_allclose(r['S'], np.diag([.9375, 2.75, 5.]))
        self.assertLess(r['congruence_error'], 1e-12)

    def test_non_spd_and_condition_guard(self):
        H = np.eye(6); H[5, 5] = -1
        with self.assertRaises(np.linalg.LinAlgError): effective(H)
        H[5, 5] = 1e-14
        with self.assertRaises(ValueError): effective(H)

    def test_uniform_includes_endpoints_not_curvature(self):
        rows = [{'stamp_ns': str(10**18 + k * 10), 'index': k} for k in range(100)]
        selected = uniform_rows(rows)
        self.assertEqual(len(selected), 20)
        self.assertEqual([selected[0]['index'], selected[-1]['index']], [0, 99])

    def test_physical_units_and_no_rotation_clipping(self):
        r = {'vectors': np.eye(3), 'X': np.eye(3)}
        p = list(perturbations(r))
        self.assertEqual(len(p), 12)
        self.assertAlmostEqual(np.linalg.norm(p[0]['eta'][:3]) * .8, .3)
        self.assertTrue(all(x['status'] == 'OUT_OF_BOUNDS' for x in p[8:]))
        self.assertGreater(p[8]['rotation_deg'], 5)


if __name__ == '__main__': unittest.main()
