import unittest
import numpy as np
from clusters import complete_link, unique_initials, competing_pair, distance


def pose(x):
    p = np.eye(4); p[0, 3] = x; return p


class ClusterTests(unittest.TestCase):
    def test_complete_link_does_not_chain_far_endpoints(self):
        p = {'a': pose(0), 'b': pose(.09), 'c': pose(.18)}
        self.assertEqual(complete_link(p), [('a', 'b'), ('c',)])

    def test_same_physical_start_not_two_ids(self):
        p = {'WEAK': pose(.3), 'COUPLED': pose(.3), 'OTHER': pose(-.3)}
        self.assertEqual(len(unique_initials(list(p), p)), 2)

    def test_single_new_minimum_not_competing_basins(self):
        p = {'a': pose(1), 'b': pose(1.01)}
        self.assertFalse(competing_pair([('a', 'b')], p, pose(0)))

    def test_two_separated_repeated_groups(self):
        p = {'a': pose(0), 'b': pose(.01), 'c': pose(1), 'd': pose(1.01)}
        self.assertTrue(competing_pair([('a', 'b'), ('c', 'd')], p, pose(0)))

    def test_float_rotation_self_distance(self):
        p = pose(0); angle=.21
        p[:2, :2] = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]], dtype=np.float32)
        self.assertLess(distance(p, p)[1], 1e-10)


if __name__ == '__main__': unittest.main()
