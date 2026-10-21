#!/usr/bin/env python3
"""Read-only verification of the inherited R15 frozen-input manifest."""

import hashlib
import json
import os
import sys


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main():
    manifest_path = sys.argv[1] if len(sys.argv) > 1 else (
        "docs/p62_r15_coupled_subspace_refinement/INPUT_FREEZE.json"
    )
    with open(manifest_path, "r", encoding="utf-8") as stream:
        manifest = json.load(stream)
    identities = manifest["inputs"]["file_identities"]
    missing, mismatches = [], []
    for identity in identities:
        path = identity["path"]
        if not os.path.isfile(path):
            missing.append(path)
            continue
        actual_bytes = os.path.getsize(path)
        actual_sha = sha256_file(path)
        if actual_bytes != identity["bytes"] or actual_sha != identity["sha256"]:
            mismatches.append({
                "path": path,
                "expected_bytes": identity["bytes"],
                "actual_bytes": actual_bytes,
                "expected_sha256": identity["sha256"],
                "actual_sha256": actual_sha,
            })
    result = {
        "manifest": manifest_path,
        "entries": len(identities),
        "missing": missing,
        "mismatches": mismatches,
        "status": "PASS" if not missing and not mismatches else "FAIL",
    }
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
