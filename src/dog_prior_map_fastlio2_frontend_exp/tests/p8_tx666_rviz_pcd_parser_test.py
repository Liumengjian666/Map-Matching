#!/usr/bin/env python3
import importlib.util
import tempfile
import unittest
from pathlib import Path

import numpy as np


SCRIPT = (Path(__file__).resolve().parents[1] / "scripts" / "p8" /
          "tx666_rviz_geometry_publisher.py")
SPEC = importlib.util.spec_from_file_location("tx666_rviz_geometry_publisher", SCRIPT)
PUBLISHER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PUBLISHER)


def write_fixture(path, xyz, trailing=b""):
    header = (
        b"# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
        b"COUNT 1 1 1\nWIDTH 2\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
        b"POINTS 2\nDATA binary\n"
    )
    path.write_bytes(header + np.asarray(xyz, dtype="<f4").tobytes() + trailing)


class PcdParserTest(unittest.TestCase):
    def test_accepts_zero_padding_after_declared_points(self):
        expected = np.array([[1, 2, 3], [-4, 5, -6]], dtype=np.float32)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "padded.pcd"
            write_fixture(path, expected, b"\x00" * 17)
            points, padding = PUBLISHER.read_binary_pcd_xyz(path)
        np.testing.assert_array_equal(points, expected)
        self.assertEqual(padding, 17)

    def test_rejects_nonzero_bytes_after_declared_points(self):
        expected = np.array([[1, 2, 3], [-4, 5, -6]], dtype=np.float32)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "invalid.pcd"
            write_fixture(path, expected, b"\x00\x01")
            with self.assertRaisesRegex(ValueError, "pcd_nonzero_trailing_payload"):
                PUBLISHER.read_binary_pcd_xyz(path)

    def test_rejects_truncated_declared_points(self):
        expected = np.array([[1, 2, 3], [-4, 5, -6]], dtype=np.float32)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "truncated.pcd"
            write_fixture(path, expected)
            path.write_bytes(path.read_bytes()[:-1])
            with self.assertRaisesRegex(ValueError, "pcd_payload_truncated"):
                PUBLISHER.read_binary_pcd_xyz(path)


if __name__ == "__main__":
    unittest.main()
