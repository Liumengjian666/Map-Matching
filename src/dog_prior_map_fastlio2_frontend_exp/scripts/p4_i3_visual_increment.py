#!/usr/bin/env python3
"""Floor01 fixed-parameter offline visual increment experiment; never plays bags."""

import argparse
import csv
import hashlib
import subprocess
import time
from pathlib import Path

import numpy as np
import rosbag
from p4_i3_visual_frontend import cv2, estimate, load_calibration, rectify, sanity

ROOT = Path(__file__).resolve().parents[1]
DATA = Path("/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01")
RUNTIME = (
    DATA
    / "results/p3_r10b_fix1_floor01_full_rerun_20260926/floor01_fix1_runtime_topics.bag"
)
MANIFEST = (
    DATA / "results/p3_r7_floor01_full_baseline/floor01_canonical_input_manifest.csv"
)
SHARDS = DATA / "raw/extracted/Multi_Floor_Rosbag"
CAM_TOPIC = "/cmu_sp1/camera_1/image_raw"
REQUEST_TOPIC = "/dog_livo/ndt/scan_request"
OUT = ROOT / "docs/p4_i3_visual_increment"
EVAL_START = 1660857393.197807074
MAX_MISMATCH_NS = 20_000_000
START_SHA = "03d38dd5a7df0c81ce1d5cf0ba5dff18c54dd451"


def write_csv(path, rows):
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def source_shards():
    # Source names come from the canonical manifest, not filename guessing.
    names = []
    with MANIFEST.open(newline="") as stream:
        for row in csv.DictReader(stream):
            if row["shard_source"] not in names:
                names.append(row["shard_source"])
    assert len(names) == 3
    return [SHARDS / name for name in names]


def camera_messages(paths):
    for path in paths:
        with rosbag.Bag(str(path)) as bag:
            yield from (
                message for _, message, _ in bag.read_messages(topics=[CAM_TOPIC])
            )


def requests():
    with rosbag.Bag(str(RUNTIME)) as bag:
        for _, message, _ in bag.read_messages(topics=[REQUEST_TOPIC]):
            assert message.scan_end_ns == message.cloud_end_frame.header.stamp.to_nsec()
            assert message.cloud_end_frame.header.frame_id == "cmu_sp1_velodyne"
            yield message


def audit():
    paths = source_shards()
    stamps = []
    camera = None
    for message in camera_messages(paths):
        metadata = {
            "topic": CAM_TOPIC,
            "type": message._type,
            "frame": message.header.frame_id,
            "encoding": message.encoding,
            "width": message.width,
            "height": message.height,
        }
        if camera is None:
            camera = metadata
        assert camera == metadata, "camera metadata changed"
        stamps.append(message.header.stamp.to_nsec())
    stamps = np.asarray(stamps, dtype=np.int64)
    assert np.all(np.diff(stamps) > 0), "camera header stamps are not monotonic"
    camera["count"] = len(stamps)
    camera["rate_hz"] = (len(stamps) - 1) * 1e9 / float(stamps[-1] - stamps[0])
    sync = []
    for request in requests():
        stamp = request.scan_end_ns
        index = int(np.searchsorted(stamps, stamp))
        candidates = [i for i in (index - 1, index) if 0 <= i < len(stamps)]
        nearest = min(candidates, key=lambda i: (abs(int(stamps[i]) - stamp), i))
        mismatch = int(stamps[nearest]) - stamp
        sync.append(
            {
                "transaction_id": request.transaction_id,
                "scan_end_ns": stamp,
                "image_index": nearest,
                "image_ns": int(stamps[nearest]),
                "signed_mismatch_ms": mismatch / 1e6,
                "abs_mismatch_ms": abs(mismatch) / 1e6,
                "eligible": int(abs(mismatch) <= MAX_MISMATCH_NS),
            }
        )
    assert len(sync) == 4127 and len(stamps) == 10017
    print("camera", camera, flush=True)
    print(
        "sync mean/median/p95/max ms",
        np.mean([r["abs_mismatch_ms"] for r in sync]),
        np.percentile([r["abs_mismatch_ms"] for r in sync], [50, 95, 100]),
        "coverage",
        np.mean([r["eligible"] for r in sync]),
        flush=True,
    )
    return paths, camera, sync


def run_visual(paths, sync, calibration, limit):
    images = iter(camera_messages(paths))
    image_index = -1
    previous = None
    rows = []
    for i, request in enumerate(requests()):
        if limit and i > limit:
            break
        match = sync[i]
        assert request.transaction_id == match["transaction_id"]
        assert request.scan_end_ns == match["scan_end_ns"]
        while image_index < match["image_index"]:
            image = next(images)
            image_index += 1
        assert image.header.stamp.to_nsec() == match["image_ns"]
        tick = time.perf_counter()
        gray = rectify(image, calibration) if match["eligible"] else None
        preprocess = (time.perf_counter() - tick) * 1000
        if previous is not None:
            old_req, old_match, old_gray, old_preprocess = previous
            row = {
                "transaction_ref": old_req.transaction_id,
                "transaction_cur": request.transaction_id,
                "timestamp_ref_ns": old_match["image_ns"],
                "timestamp_cur_ns": match["image_ns"],
                "scan_ref_ns": old_req.scan_end_ns,
                "scan_cur_ns": request.scan_end_ns,
                "eval_time_s": request.scan_end_ns * 1e-9 - EVAL_START,
                "mismatch_ref_ms": old_match["signed_mismatch_ms"],
                "mismatch_cur_ms": match["signed_mismatch_ms"],
                "image_dt_s": (match["image_ns"] - old_match["image_ns"]) * 1e-9,
                "scan_dt_s": (request.scan_end_ns - old_req.scan_end_ns) * 1e-9,
                "attempted": int(gray is not None and old_gray is not None),
                "status": "SYNC_INVALID",
                "detected": 0,
                "klt_valid": 0,
                "klt_forward_valid": 0,
                "fb_valid": 0,
                "depth_associated": 0,
                "pnp_correspondences": 0,
                "pnp_inliers": 0,
                "inlier_ratio": float("nan"),
                "reprojection_rmse_px": float("nan"),
                "feature_ms": 0.0,
                "klt_ms": 0.0,
                "depth_ms": 0.0,
                "projection_ms": 0.0,
                "association_ms": 0.0,
                "pnp_ms": 0.0,
                "preprocess_ms": preprocess + old_preprocess,
                "total_ms": 0.0,
            }
            transform = None
            if row["attempted"]:
                assert match["image_index"] > old_match["image_index"]
                tick = time.perf_counter()
                metrics, transform = estimate(
                    old_gray,
                    gray,
                    old_req.cloud_end_frame,
                    calibration,
                    request.transaction_id,
                )
                row.update(metrics)
                # Count both rectifications conservatively even though reference is cached.
                row["total_ms"] = (time.perf_counter() - tick) * 1000 + row[
                    "preprocess_ms"
                ]
            for a in range(3):
                for b in range(4):
                    row[f"T_Ccur_Cref_{a}{b}"] = (
                        float("nan") if transform is None else float(transform[a, b])
                    )
            rows.append(row)
        previous = (request, match, gray, preprocess)
        if i and i % 200 == 0:
            print(
                "processed",
                i,
                "valid",
                sum(r["status"] == "VALID" for r in rows),
                flush=True,
            )
    return rows


def sha256(path):
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            value.update(block)
    return value.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--audit-only", action="store_true")
    parser.add_argument(
        "--limit", type=int, default=0, help="implementation smoke only; zero=full"
    )
    parser.add_argument("--out", type=Path, default=OUT)
    args = parser.parse_args()
    workspace = ROOT.parents[1]
    head = subprocess.check_output(
        ["git", "-C", str(workspace), "rev-parse", "HEAD"], text=True
    ).strip()
    assert head in (START_SHA, "8def495a82245d52772c3bc78334adfaf797cce9"), (
        "unexpected experiment starting revision"
    )
    cv2.setNumThreads(1)
    calibration = load_calibration(DATA / "calibration")
    sanity(calibration)
    paths, camera, sync = audit()
    args.out.mkdir(parents=True, exist_ok=True)
    write_csv(args.out / "sync_stats.csv", sync)
    print("raw shards:", *[str(p) for p in paths], sep="\n", flush=True)
    if not args.audit_only:
        if np.mean([r["eligible"] for r in sync]) < 0.5:
            raise RuntimeError(
                "SYNC_COVERAGE_BELOW_50_PERCENT; visual experiment not started"
            )
        rows = run_visual(paths, sync, calibration, args.limit)
        # Freeze measurements before loading GT or existing predictor errors.
        write_csv(args.out / "visual_increment.csv", rows)
        from p4_i3_visual_report import evaluate_and_report

        evaluate_and_report(
            rows, sync, camera, calibration, paths, args.out, args.limit
        )


if __name__ == "__main__":
    main()
