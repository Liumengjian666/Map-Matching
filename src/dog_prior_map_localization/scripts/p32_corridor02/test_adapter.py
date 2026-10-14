#!/usr/bin/env python3
"""Small data-interface tests, not localization or GT tests."""
import unittest
from types import SimpleNamespace

import numpy as np

from prepare_probe import bounded_messages, cloud_array, proper_transform


class AdapterTests(unittest.TestCase):
    def test_rounded_rotation_preserves_translation(self):
        matrix = np.eye(4)
        matrix[:3, 3] = [1, 2, 3]
        matrix[0, 0] += 1e-6
        fixed, change = proper_transform(matrix)
        np.testing.assert_allclose(fixed[:3, :3].T@fixed[:3, :3], np.eye(3), atol=1e-12)
        np.testing.assert_array_equal(fixed[:3, 3], matrix[:3, 3])
        self.assertGreater(change, 0)
        self.assertEqual(matrix[0, 0], 1+1e-6)  # No input mutation.

    def test_invalid_transform_rejected(self):
        for matrix in (np.zeros((4, 4)), np.full((4, 4), np.nan), np.diag([-1., 1, 1, 1])):
            with self.assertRaises(ValueError):
                proper_transform(matrix)

    def test_unaligned_float_time_and_row_padding(self):
        names = ('x', 'y', 'z', 'time')
        offsets = (0, 4, 8, 18)
        for endian in ('<', '>'):
            dtype = np.dtype(dict(names=names, formats=[endian+'f4']*4, offsets=offsets, itemsize=22))
            data = bytearray(48)
            array = np.ndarray((2, 1), dtype=dtype, buffer=data, strides=(24, 22))
            for i, name in enumerate(names):
                array[name][:, 0] = [i+.25, i+.75]
            message = SimpleNamespace(fields=[SimpleNamespace(name=n, offset=o, datatype=7, count=1)
                for n, o in zip(names, offsets)], is_bigendian=endian=='>', point_step=22,
                row_step=24, width=1, height=2, data=data)
            expected = np.array([[.25, 1.25, 2.25, 3.25], [.75, 1.75, 2.75, 3.75]])
            np.testing.assert_array_equal(cloud_array(message), expected)

    def test_point_layout_rejected(self):
        with self.assertRaisesRegex(ValueError, 'EXPECTED_FLOAT32'):
            cloud_array(SimpleNamespace(fields=[]))

    def test_missing_modality_cannot_expand_traversal(self):
        self.assertEqual(list(bounded_messages(range(3), limit=3)), [0, 1, 2])
        with self.assertRaisesRegex(ValueError, 'BOUNDED_PREFIX'):
            list(bounded_messages(range(4), limit=3))


if __name__ == '__main__':
    unittest.main()
