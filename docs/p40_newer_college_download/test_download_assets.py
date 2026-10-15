"""Small offline safety tests; never download or access dataset payloads."""
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
import zipfile
from unittest.mock import patch

import download_assets as downloader
from prepare_pointclouds import validate_member


class DownloadTests(unittest.TestCase):
    def test_streaming_digest_and_size_validation(self):
        with tempfile.TemporaryDirectory() as name:
            path = Path(name) / 'data.csv'
            path.write_bytes(b'1,2,3\n')
            result = downloader.validate(path, {'bytes': 6})
            self.assertEqual(result['sha256'], hashlib.sha256(b'1,2,3\n').hexdigest())
            with self.assertRaisesRegex(RuntimeError, 'Size mismatch'):
                downloader.validate(path, {'bytes': 7})

    def test_html_is_not_accepted_as_payload(self):
        with tempfile.TemporaryDirectory() as name:
            path = Path(name) / 'map.ply.part'
            data = b'<!DOCTYPE html><html>Sign in</html>'
            path.write_bytes(data)
            with self.assertRaisesRegex(RuntimeError, 'HTML'):
                downloader.validate(path, {'bytes': len(data)})

    def test_zip_magic_is_checked(self):
        with tempfile.TemporaryDirectory() as name:
            path = Path(name) / 'ouster_scan-001.zip.part'
            path.write_bytes(b'not a zip')
            with self.assertRaisesRegex(RuntimeError, 'ZIP signature'):
                downloader.validate(path, {'bytes': 9})

    def test_missing_external_mount_is_rejected(self):
        with patch.object(downloader.subprocess, 'check_output', return_value='wrong-uuid'):
            with self.assertRaisesRegex(RuntimeError, 'not mounted'):
                downloader.check_mount()

    def test_root_symlink_is_rejected_before_chdir(self):
        with tempfile.TemporaryDirectory() as name:
            target = Path(name) / 'external-root'
            target.symlink_to(Path(name), target_is_directory=True)
            plan = Path(name) / 'plan.json'
            plan.write_text(json.dumps({'files': []}))
            with patch.object(downloader, 'ROOT', target), \
                    patch.object(downloader, 'check_mount'), \
                    patch.object(downloader.os, 'chdir') as chdir:
                with self.assertRaisesRegex(RuntimeError, 'symlinks'):
                    downloader.main(str(plan))
                chdir.assert_not_called()

    def test_remounted_device_must_match_pinned_directory(self):
        with patch.object(downloader.subprocess, 'check_output', return_value='F627-4133'), \
                patch.object(downloader.os.path, 'ismount', return_value=True), \
                patch.object(downloader.os, 'stat') as stat:
            stat.side_effect = [type('Stat', (), {'st_dev': 10})(),
                                type('Stat', (), {'st_dev': 20})()]
            with self.assertRaisesRegex(RuntimeError, 'another filesystem'):
                downloader.check_mount(10)

    def test_plan_has_unique_external_relative_destinations(self):
        plan = json.loads(Path(__file__).with_name('download_plan.json').read_text())
        paths = [e['path'] for e in plan['files']]
        self.assertEqual(len(paths), len(set(paths)))
        for path in paths:
            self.assertFalse(Path(path).is_absolute())
            self.assertNotIn('..', Path(path).parts)

    def test_archive_rejects_traversal_symlinks_and_duplicate_scans(self):
        names = set()
        good = zipfile.ZipInfo('ouster_scan/cloud_1583836591_182590976.pcd')
        validate_member(good, names)
        validate_member(zipfile.ZipInfo('ouster_scan/cloud_1583836591_182590976(1).pcd'), names)
        with self.assertRaisesRegex(RuntimeError, 'Duplicate'):
            validate_member(good, names)
        for name in ['../bad.pcd', '/tmp/bad.pcd', 'ouster_scan/../bad.pcd']:
            with self.assertRaisesRegex(RuntimeError, 'Unexpected'):
                validate_member(zipfile.ZipInfo(name), set())
        symbolic = zipfile.ZipInfo('ouster_scan/cloud_1583836591_182590976.pcd')
        symbolic.external_attr = 0o120777 << 16
        with self.assertRaisesRegex(RuntimeError, 'Unexpected'):
            validate_member(symbolic, set())


if __name__ == '__main__':
    unittest.main()
