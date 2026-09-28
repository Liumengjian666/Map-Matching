#!/usr/bin/env python3
"""Exercise frozen selection hash rejection before any NDT alignment."""

import os
import pathlib
import shutil
import subprocess
import sys
import tempfile


def run_rejected_case(binary, package_root, field_name):
    frozen_dir = (
        pathlib.Path(package_root)
        / "docs"
        / "p6_i5c_targeted_terminal_audit"
    )
    frozen_selection = frozen_dir / "selected_cases.csv"
    frozen_manifest = frozen_dir / "selection_manifest.json"

    with tempfile.TemporaryDirectory(prefix="p6-i5c-selection-guard-") as temp:
        temp_root = pathlib.Path(temp)
        output_dir = temp_root / "output"
        output_dir.mkdir()
        selected_path = temp_root / "selected_cases.csv"
        manifest_path = output_dir / "selection_manifest.json"
        shutil.copyfile(frozen_selection, selected_path)
        shutil.copyfile(frozen_manifest, manifest_path)

        tampered_path = selected_path if field_name == "selected_cases" else manifest_path
        with tampered_path.open("ab") as stream:
            stream.write(b"\n")

        env = os.environ.copy()
        env["OMP_NUM_THREADS"] = "1"
        # The interactive ROS shell prepends the camera SDK's older libusb,
        # which cannot satisfy this system PCL 1.10 build. This offline audit
        # executable uses system dependencies only.
        env.pop("LD_LIBRARY_PATH", None)
        completed = subprocess.run(
            [
                str(binary),
                "smoke",
                str(selected_path),
                str(temp_root / "unused_scan_assets.csv"),
                str(temp_root / "unused_packed_xyz.bin"),
                str(temp_root / "unused_map.pcd"),
                str(output_dir),
            ],
            env=env,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )
        expected_error = (
            "FROZEN_SELECTION_HASH_MISMATCH"
            if field_name == "selected_cases"
            else "FROZEN_SELECTION_MANIFEST_HASH_MISMATCH"
        )
        output = completed.stdout + completed.stderr
        if completed.returncode == 0 or expected_error not in output:
            raise AssertionError(
                f"{field_name} mutation was not rejected as expected: "
                f"status={completed.returncode}, output={output!r}"
            )
        if (output_dir / "call_accounting.csv").exists():
            raise AssertionError("selection mismatch reached NDT call accounting")
        if (output_dir / "replay_parity.csv").exists():
            raise AssertionError("selection mismatch reached replay output")


def main():
    if len(sys.argv) != 3:
        raise SystemExit("usage: test_p6_i5c_frozen_selection_guard.py AUDIT_BIN PACKAGE_ROOT")
    binary = pathlib.Path(sys.argv[1]).resolve()
    package_root = pathlib.Path(sys.argv[2]).resolve()
    run_rejected_case(binary, package_root, "selected_cases")
    run_rejected_case(binary, package_root, "selection_manifest")
    print("PASS:frozen_selection_and_manifest_hash_mismatch_rejected_before_ndt")


if __name__ == "__main__":
    main()
