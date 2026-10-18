#!/usr/bin/env python3
"""Focused regressions for the real launch-path failure and asset preservation."""
import importlib.util
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from rospy import myargv
import roslib.packages


WORKSPACE = Path(__file__).resolve().parents[2]
SCRIPT = WORKSPACE / "src/dog_prior_map_localization/scripts/p60_raw_registration_probe.py"
spec = importlib.util.spec_from_file_location("p60_probe", SCRIPT)
probe = importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe)


class AdapterTests(unittest.TestCase):
    def test_ros_can_resolve_package_executable(self):
        self.assertEqual(roslib.packages.find_node("dog_prior_map_localization",
                         "p60_raw_registration_probe.py"), [str(SCRIPT)])

    def test_ros_remaps_do_not_reach_argparse(self):
        self.assertEqual(myargv(["probe", "publish", "--bag", "internal.bag",
                                 "__name:=test", "__log:=internal.log"]),
                         ["probe", "publish", "--bag", "internal.bag"])

    def test_prepare_refuses_existing_asset_directory(self):
        with tempfile.TemporaryDirectory() as root:
            with self.assertRaises(FileExistsError):
                probe.prepare("must-not-be-opened.bag", root)

    def test_changed_map_is_rejected_before_ros_initialization(self):
        with tempfile.TemporaryDirectory() as root:
            pcd = Path(root) / "first_scan_xyz.pcd"
            pcd.write_bytes(b"frozen map fixture")
            manifest = {"map_path": str(pcd),
                        "map_sha256": hashlib.sha256(pcd.read_bytes()).hexdigest()}
            (Path(root) / "map_manifest.json").write_text(json.dumps(manifest))
            self.assertEqual(probe.load_frozen_map(root), manifest)
            pcd.write_bytes(b"changed map fixture")
            with self.assertRaisesRegex(RuntimeError, "hash mismatch"):
                probe.load_frozen_map(root)


if __name__ == "__main__":
    unittest.main()
