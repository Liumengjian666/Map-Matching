#!/usr/bin/env python3
"""Freeze replay artifacts and runtime provenance before any GT is read."""

import argparse
import hashlib
import json
import subprocess
from datetime import datetime, timezone
from pathlib import Path


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def git(repo, *args):
    return subprocess.check_output(["git", "-C", str(repo), *args], text=True).strip()


def file_receipt(path, root=None):
    path = Path(path).resolve(strict=True)
    item = {"path": str(path), "bytes": path.stat().st_size, "sha256": sha256(path)}
    if root is not None:
        try:
            item["relative_to_repository"] = str(path.relative_to(root.resolve()))
        except ValueError:
            pass
    return item


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", required=True, type=Path)
    parser.add_argument("--run-root", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--v1-bag", required=True, type=Path)
    parser.add_argument("--v1-manifest", required=True, type=Path)
    parser.add_argument("--v2-bag", required=True, type=Path)
    parser.add_argument("--v2-manifest", required=True, type=Path)
    parser.add_argument("--map", required=True, type=Path)
    parser.add_argument("--extrinsics", required=True, type=Path)
    parser.add_argument("--initializer", required=True, type=Path)
    parser.add_argument("--raw-imu", required=True, type=Path)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--base-config", required=True, type=Path)
    parser.add_argument("--launch", required=True, type=Path)
    parser.add_argument("--ndt-binary", required=True, type=Path)
    parser.add_argument("--ekf-binary", required=True, type=Path)
    parser.add_argument("--audit", required=True, type=Path)
    parser.add_argument("--parity", required=True, type=Path)
    args = parser.parse_args()

    if args.output.exists():
        raise SystemExit("refusing_to_overwrite_existing_pre_gt_freeze")
    repo = args.repo.resolve(strict=True)
    run_root = args.run_root.resolve(strict=True)
    audit = json.loads(args.audit.read_text())
    parity = json.loads(args.parity.read_text())
    if audit.get("GT_LOADED") is not False or audit.get("structural_result") != "PASS":
        raise SystemExit("pre_gt_structural_audit_not_passed")
    if parity.get("result") != "PASS" or parity.get("GT_LOADED") is not False:
        raise SystemExit("control_shadow_parity_not_passed")

    output_inventory = {}
    for path in sorted(p for p in run_root.rglob("*") if p.is_file()):
        if path.resolve() == args.output.resolve():
            continue
        rel = str(path.relative_to(run_root))
        output_inventory[rel] = file_receipt(path)

    source_paths = set()
    for rel in git(repo, "diff", "--name-only", "--", "src/dog_prior_map_localization").splitlines():
        if rel:
            source_paths.add(rel)
    for rel in git(repo, "ls-files", "--others", "--exclude-standard", "--", "src/dog_prior_map_localization").splitlines():
        if rel and Path(repo / rel).is_file():
            source_paths.add(rel)
    source_inventory = {rel: file_receipt(repo / rel, repo) for rel in sorted(source_paths)}
    binary_receipts = {
        "ndt": file_receipt(args.ndt_binary, repo),
        "ekf": file_receipt(args.ekf_binary, repo),
    }

    required_run_dirs = ("control_v2", "shadow_v2", "weak_only_feedback_v2", "coupled_feedback_v2")
    missing = [name for name in required_run_dirs if not (run_root / name).is_dir()]
    if missing:
        raise SystemExit("missing_run_directories:" + ",".join(missing))

    receipt = {
        "protocol": "P10_CORRIDOR01_DETERMINISTIC_OOSM_REPLAY_V2",
        "frozen_at_utc": datetime.now(timezone.utc).isoformat(),
        "GT_LOADED": False,
        "evaluation_contract_frozen_before_GT": {
            "fit_stream": "CONTROL_V2 IMU state lineage IMU_HIGH_RATE_PUBLISH",
            "fit_interval": "first 3.0 seconds beginning at first NDT sensor stamp + 5.0 seconds startup",
            "shared_transform": True,
            "primary_metrics_interval": "evaluation origin + 3.0 seconds through +35.0 seconds",
            "metric_interpretation": "PREFIX-ALIGNED RELATIVE DRIFT; not absolute map-frame ATE",
        },
        "repository": {
            "branch": git(repo, "branch", "--show-current"),
            "head": git(repo, "rev-parse", "HEAD"),
            "working_tree_diff_sha256": hashlib.sha256(
                subprocess.check_output(["git", "-C", str(repo), "diff", "--binary", "HEAD", "--", "src/dog_prior_map_localization"])
            ).hexdigest(),
            "runtime_source_files": source_inventory,
        },
        "runtime_binaries": binary_receipts,
        "frozen_inputs": {
            "v1_bag": file_receipt(args.v1_bag),
            "v1_manifest": file_receipt(args.v1_manifest),
            "v2_bag": file_receipt(args.v2_bag),
            "v2_manifest": file_receipt(args.v2_manifest),
            "normalized_map": file_receipt(args.map),
            "extrinsics": file_receipt(args.extrinsics),
            "sensor_only_initializer": file_receipt(args.initializer),
            "source_imu_csv": file_receipt(args.raw_imu),
            "dataset_config": file_receipt(args.config, repo),
            "base_config": file_receipt(args.base_config, repo),
            "split_launch": file_receipt(args.launch, repo),
        },
        "pre_gt_audit": file_receipt(args.audit),
        "control_shadow_parity": file_receipt(args.parity),
        "frozen_run_output_inventory": output_inventory,
        "limitations": [
            "No GT path or GT bytes are included in this pre-evaluation freeze.",
            "Coupled feedback internal EKF lineage is complete (396 NDT correction events and 8004 high-rate publish events); its rosbag is partial and only the recorded prefix was pose-checked.",
            "Peak RSS was not captured during these runs.",
        ],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(receipt, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps({"result": "PASS", "freeze": str(args.output),
                      "output_files": len(output_inventory),
                      "runtime_source_files": len(source_inventory),
                      "GT_LOADED": False}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
