import hashlib
import signal
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import acquire_assets as acquisition


class AcquisitionTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix='p62_g0_test_', dir='/home/jian/livox_ws')
        self.root = Path(self.temporary.name)
        self.payload = b'#ROSBAG V2.0\nsynthetic'
        self.digest = hashlib.sha256(self.payload).hexdigest()
        cache = self.root / 'cache'
        cache.mkdir()
        (cache / 'tiny').write_bytes(self.payload)
        self.patches = [patch.object(acquisition, 'ROOT', self.root),
                        patch.object(acquisition, 'CACHE', cache),
                        patch.object(acquisition, 'ASSETS', [('tiny.bag', len(self.payload), self.digest, b'#ROSBAG', 'tiny')]),
                        patch.object(acquisition, 'available', return_value=50_000_000_000),
                        patch.object(acquisition.urllib.request, 'urlopen', side_effect=AssertionError('NO_NETWORK_IN_TEST'))]
        for item in self.patches:
            item.start()

    def tearDown(self):
        signal.setitimer(signal.ITIMER_REAL, 0)
        for item in reversed(self.patches):
            item.stop()
        self.temporary.cleanup()

    def test_verified_cache_published_and_partial_preserved(self):
        acquisition.main()
        self.assertEqual((self.root/'assets/tiny.bag').read_bytes(), self.payload)
        self.assertTrue((self.root/'staging/tiny.bag.part').exists())

    def test_insufficient_storage_stops_without_receipt(self):
        with patch.object(acquisition, 'available', return_value=47_999_999_999):
            with self.assertRaisesRegex(RuntimeError, 'G0_BLOCKED_STORAGE'):
                acquisition.main()
        self.assertFalse((self.root/'acquisition.json').exists())

    def test_existing_partial_never_overwritten(self):
        (self.root/'staging').mkdir()
        old = self.root/'staging/tiny.bag.part'; old.write_bytes(b'old')
        with self.assertRaisesRegex(RuntimeError, 'PARTIAL_EXISTS'):
            acquisition.main()
        self.assertEqual(old.read_bytes(), b'old')

    def test_existing_conflicting_asset_stops(self):
        (self.root/'assets').mkdir()
        old = self.root/'assets/tiny.bag'; old.write_bytes(b'wrong')
        with self.assertRaisesRegex(RuntimeError, 'IDENTITY_MISMATCH'):
            acquisition.main()
        self.assertEqual(old.read_bytes(), b'wrong')

    def test_existing_verified_asset_not_changed(self):
        (self.root/'assets').mkdir()
        old = self.root/'assets/tiny.bag'; old.write_bytes(self.payload)
        before = old.stat().st_mtime_ns
        acquisition.main()
        self.assertEqual(before, old.stat().st_mtime_ns)

    def test_wrong_cache_sha_rejected(self):
        (self.root/'cache/tiny').write_bytes(b'wrong')
        with self.assertRaisesRegex(RuntimeError, 'IDENTITY_MISMATCH'):
            acquisition.main()
        self.assertFalse((self.root/'assets/tiny.bag').exists())

    def test_deadline_handler_interrupts(self):
        with self.assertRaises(TimeoutError):
            acquisition.deadline(None, None)

    def test_symlinked_directory_rejected(self):
        (self.root/'alias').symlink_to('/tmp', target_is_directory=True)
        with self.assertRaisesRegex(RuntimeError, 'NON_INTERNAL_DIRECTORY'):
            acquisition.internal_directory(self.root/'alias')

    def test_pending_write_cannot_cross_reserve(self):
        reserve = 20 * 1024**3
        with patch.object(acquisition, 'available', side_effect=[50_000_000_000, 50_000_000_000, reserve+1, reserve+1]):
            with self.assertRaisesRegex(RuntimeError, 'SAFETY_RESERVE'):
                acquisition.main()
        self.assertEqual((self.root/'staging/tiny.bag.part').stat().st_size, 0)
        self.assertFalse((self.root/'assets/tiny.bag').exists())

    def test_symlink_parent_cannot_create_child_before_rejection(self):
        outside = self.root/'fixture'; outside.mkdir()
        (self.root/'assets').symlink_to(outside, target_is_directory=True)
        with self.assertRaisesRegex(RuntimeError, 'NON_INTERNAL_DIRECTORY'):
            acquisition.internal_directory(self.root/'assets/not_created')
        self.assertFalse((outside/'not_created').exists())


if __name__ == '__main__':
    unittest.main()
