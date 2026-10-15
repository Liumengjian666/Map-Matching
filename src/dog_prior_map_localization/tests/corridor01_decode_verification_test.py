#!/usr/bin/env python3
"""Small unit checks; no dataset, ROS master, map or GT is read."""
import importlib.util
from pathlib import Path
import struct
import tempfile
import unittest

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/corridor01_decode/verify_full.py'
SPEC = importlib.util.spec_from_file_location('verify_full', SCRIPT)
VERIFY = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(VERIFY)


class DecodeVerificationTest(unittest.TestCase):
    def test_header_integer_nanoseconds(self):
        self.assertEqual(VERIFY.header_ns(struct.pack('<IIII', 1, 1517157219, 188978910, 0)),
                         1517157219188978910)

    def test_invalid_header_rejected(self):
        for value in (b'', struct.pack('<IIII', 1, 0, 0, 0),
                      struct.pack('<IIII', 1, 1517157219, 1000000000, 0)):
            with self.subTest(value=value), self.assertRaises(RuntimeError):
                VERIFY.header_ns(value)

    def test_rich_point_layout(self):
        self.assertEqual(VERIFY.RICH_DTYPE.itemsize, 24)
        self.assertEqual(VERIFY.RICH_DTYPE.fields['ring'][1], 16)
        self.assertEqual(VERIFY.RICH_DTYPE.fields['time'][1], 20)

    def test_existing_output_is_never_overwritten(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            partial, final = path / 'input.partial', path / 'existing.bag'
            final.write_bytes(b'keep')
            with self.assertRaisesRegex(RuntimeError, 'refusing_existing_output'):
                VERIFY.verify(partial, final, path / 'unused_decoder')
            self.assertEqual(final.read_bytes(), b'keep')

    def test_publish_renames_the_same_file(self):
        with tempfile.TemporaryDirectory() as directory:
            source, target = Path(directory) / 'source.partial', Path(directory) / 'target.bag'
            source.write_bytes(b'complete')
            inode = source.stat().st_ino
            VERIFY.publish_no_clobber(source, target)
            self.assertFalse(source.exists())
            self.assertEqual(target.read_bytes(), b'complete')
            self.assertEqual(target.stat().st_ino, inode)

    def test_publish_keeps_existing_destination_and_source(self):
        with tempfile.TemporaryDirectory() as directory:
            source, target = Path(directory) / 'source.partial', Path(directory) / 'target.bag'
            source.write_bytes(b'new')
            target.write_bytes(b'keep')
            with self.assertRaisesRegex(RuntimeError, 'refusing_existing_output'):
                VERIFY.publish_no_clobber(source, target)
            self.assertEqual(source.read_bytes(), b'new')
            self.assertEqual(target.read_bytes(), b'keep')


if __name__ == '__main__':
    unittest.main(verbosity=2)
