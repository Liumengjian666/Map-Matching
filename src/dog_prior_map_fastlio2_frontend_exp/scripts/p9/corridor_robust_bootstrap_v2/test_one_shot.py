"""Synthetic subprocess fixtures: never execute a real alignment."""
import contextlib
import io
import json
import pathlib
import subprocess
import tempfile
import unittest
from unittest.mock import patch
import execute_v2_once as entry

class OneShotTests(unittest.TestCase):
    def check_repeat(self, returncode):
        with tempfile.TemporaryDirectory(prefix="p9_v2_guard_test.") as scratch:
            root = pathlib.Path(scratch)
            (root / "quality_gate_freeze.json").write_text("{}\n")
            verified = ({"binary_sha256": "SYNTHETIC"}, {"execution_environment": {}}, ["NEVER_EXECUTED_FAKE_ALIGN"])
            with patch.object(entry, "OUTPUT", root), patch.object(entry, "ARCHIVE", root), \
                 patch.object(entry, "verified_command", return_value=verified), \
                 patch.object(entry.subprocess, "run", return_value=subprocess.CompletedProcess([], returncode)) as child, \
                 contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(entry.execute(), returncode)
                with self.assertRaises(FileExistsError):
                    entry.execute()
                self.assertEqual(child.call_count, 1)
            receipt = json.loads((root / "V2_ALIGNMENT_COMPLETED.json").read_text())
            self.assertFalse(receipt["scientific_repeats_permitted"])

    def test_success_cannot_repeat_by_changing_child_path(self):
        self.check_repeat(0)

    def test_crash_also_consumes_one_shot_no_silent_retry(self):
        self.check_repeat(1)

if __name__ == "__main__":
    unittest.main()
