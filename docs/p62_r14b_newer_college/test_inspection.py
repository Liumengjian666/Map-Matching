"""Synthetic parser guard tests, not PCL or localization evidence."""
from pathlib import Path
import struct
import tempfile
import unittest
from inspect_inputs import pcd_sample


def data(extra=b'', encoding='binary', time_type='U'):
    header = ('VERSION 0.7\nFIELDS x y z t\nSIZE 4 4 4 4\n'
              f'TYPE F F F {time_type}\nCOUNT 1 1 1 1\nWIDTH 1\nHEIGHT 1\n'
              f'POINTS 1\nDATA {encoding}\n').encode()
    return header + struct.pack('<fffI', 1., 2., 3., 123) + extra


class InspectionTests(unittest.TestCase):
    def inspect(self, value):
        with tempfile.TemporaryDirectory(prefix='r14b_parser_test_') as folder:
            path = Path(folder) / 'synthetic.pcd'
            path.write_bytes(value)
            return pcd_sample(path)

    def test_exact_binary(self):
        out = self.inspect(data())
        self.assertEqual(out['finite_xyz_count'], 1)
        self.assertEqual((out['t_min'], out['t_max']), (123, 123))

    def test_zero_suffix_does_not_claim_decode_pass(self):
        out = self.inspect(data(b'\0' * 13))
        self.assertEqual(out['trailing_bytes'], 13)
        self.assertTrue(out['trailing_all_zero'])
        self.assertIsNone(out['finite_xyz_count'])
        self.assertEqual(out['decode_status'], 'NONCANONICAL_LENGTH_NEEDS_INDEPENDENT_PCL_READ')

    def test_nonzero_suffix_visible(self):
        out = self.inspect(data(b'\x01'))
        self.assertFalse(out['trailing_all_zero'])
        self.assertIsNone(out['t_min'])

    def test_truncated_data_not_passed(self):
        out = self.inspect(data()[:-2])
        self.assertEqual(out['decode_status'], 'NONCANONICAL_LENGTH_NEEDS_INDEPENDENT_PCL_READ')
        self.assertIsNone(out['finite_xyz_count'])

    def test_compressed_not_guessed(self):
        with self.assertRaises(ValueError):
            self.inspect(data(encoding='binary_compressed'))

    def test_time_storage_not_guessed(self):
        with self.assertRaises(ValueError):
            self.inspect(data(time_type='F'))


if __name__ == '__main__':
    unittest.main(verbosity=2)
