#!/usr/bin/env python3
"""Create causal metric LK/PnP factors from Corridor01's pinned v2 inputs.

Depth is the nearest rotationally-deskewed LiDAR cloud at or before the
reference image sensor timestamp. Bag record timestamps are never consulted.
The PnP translation is conjugated through the official camera/IMU extrinsic;
only its relative metric translation is exported, never visual rotation.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.util
import sys
from collections import deque
from pathlib import Path

import numpy as np


REPO = Path(__file__).resolve().parents[3]
PACKAGE = REPO / "src/dog_prior_map_fastlio2_frontend_exp"
FRONTEND = PACKAGE / "scripts/p4_i3_visual_frontend.py"
ROOT = Path("/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01")
RAW_BAG = ROOT / "raw/Long_Corridor_Rosbag/raw_data_core_2023-07-25-03-01-44.bag"
DERIVED_BAG = ROOT / "derived/corridor01_adapted_full_se3_v2.bag"
CALIB_DIR = ROOT / "calibration"
INPUT_DIR = ROOT / "results/p6_i6c_framework/input"
INPUT_MANIFEST = INPUT_DIR / "input_manifest.txt"
OUTPUT = PACKAGE / "docs/p6_i6d_full_algorithm/corridor01_metric_visual_causal.csv"
IMAGE_TOPIC = "/camera_1/image_raw"
CLOUD_TOPIC = "/superloc_adapter/points_rot_only"
EXPECTED_MANIFEST_SHA256 = "6d722ec6946570cc09d984c1a8ac7ebaaff799f012f9386ae169e3d47747a043"
EXPECTED_RAW_SHA256 = "c9e127402463eebee25c54bc967eb377e026bbbdfc979eae853a5af134180811"
EXPECTED_DERIVED_SHA256 = "7c52b3703f2f5f9b7fe291e579187c67c0181e015df6a9a8398cf4795b547ba0"
EXPECTED_INTRINSICS_SHA256 = "083ff73553f6df25734bfddc439fbda7eb7b01c8baeede2b4949e960f72fd370"
EXPECTED_EXTRINSICS_SHA256 = "59b02c1fe6103196ec46645c960f3908d092c0a4ba7d93c22762bcd61210b87d"
EVAL_START_NS = 1_517_157_224_188_979_000
MAX_DEPTH_AGE_NS = 20_000_000
MAX_PAIR_GAP_NS = 250_000_000


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def stamp_ns(message) -> int:
    stamp = message.header.stamp
    return int(stamp.secs) * 1_000_000_000 + int(stamp.nsecs)


def load_frontend():
    spec = importlib.util.spec_from_file_location("p4_i3_visual_frontend", FRONTEND)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot_load_frozen_P4_I3_visual_frontend")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    parser.add_argument("--max-pairs", type=int,
                        help="optional bounded smoke; full run is the default")
    args = parser.parse_args()

    for path in (RAW_BAG, DERIVED_BAG, INPUT_MANIFEST,
                 CALIB_DIR / "corridor01_intrinsics.yaml",
                 CALIB_DIR / "corridor01_extrinsics.yaml"):
        if not path.is_file():
            raise FileNotFoundError(path)
    if sha256(INPUT_MANIFEST) != EXPECTED_MANIFEST_SHA256:
        raise RuntimeError("Corridor01 v2 input manifest SHA-256 mismatch")
    if sha256(RAW_BAG) != EXPECTED_RAW_SHA256 or sha256(DERIVED_BAG) != EXPECTED_DERIVED_SHA256:
        raise RuntimeError("Corridor01 source bag identity mismatch")
    if sha256(CALIB_DIR / "corridor01_intrinsics.yaml") != EXPECTED_INTRINSICS_SHA256 or \
       sha256(CALIB_DIR / "corridor01_extrinsics.yaml") != EXPECTED_EXTRINSICS_SHA256:
        raise RuntimeError("Corridor01 official calibration identity mismatch")

    # Import the frozen P4 frontend first. It deliberately loads the distro
    # OpenCV build with ccalib/omnidir; importing cv_bridge first can populate
    # sys.modules['cv2'] from ~/.local (which lacks omnidir), defeating that
    # loader contract. This script reads sensor messages directly and does not
    # need CvBridge.
    vision = load_frontend()
    calibration = vision.load_calibration(CALIB_DIR, "corridor01")
    import rosbag

    args.output.parent.mkdir(parents=True, exist_ok=True)
    temp_output = args.output.with_suffix(args.output.suffix + ".partial")
    columns = ["transaction_ref", "transaction_cur", "ref_ns", "cur_ns",
               "depth_stamp_ns", "zx", "zy", "zz", "inliers",
               "inlier_ratio", "reprojection_rmse_px", "status"]
    stats = {"image_messages": 0, "cloud_messages": 0, "pairs": 0,
             "depth_unsynchronized": 0, "pair_gap_rejected": 0,
             "pnp_valid": 0, "pnp_invalid": 0}
    image_queue = deque()
    cloud_queue = deque()
    last_image_stamp = 0
    last_cloud_stamp = 0
    latest_cloud_stamp = 0
    pair_index = 0

    def encode_pair(ref_image, cur_image, depth_message, writer):
        ref_stamp, ref_gray = ref_image
        cur_stamp, cur_gray = cur_image
        if cur_stamp - ref_stamp > MAX_PAIR_GAP_NS:
            stats["pair_gap_rejected"] += 1
            return
        if ref_stamp < EVAL_START_NS:
            return
        result, transform_camera_cur_camera_ref = vision.estimate(
            ref_gray, cur_gray, depth_message, calibration, 60101 + pair_index)
        if transform_camera_cur_camera_ref is None:
            stats["pnp_invalid"] += 1
            return
        # P4-I3's verified direction: T_Ccur_Cref maps ref-camera points into
        # current-camera coordinates. Inverting and conjugating gives the
        # reference-to-current IMU increment used by the map-frame anchor.
        t_imu_camera = calibration["T_imu_camera"]
        t_imu_cur_imu_ref = (t_imu_camera @
                             np.linalg.inv(transform_camera_cur_camera_ref) @
                             np.linalg.inv(t_imu_camera))
        translation = t_imu_cur_imu_ref[:3, 3]
        inliers = int(result["pnp_inliers"])
        ratio = float(result["inlier_ratio"])
        reprojection = float(result["reprojection_rmse_px"])
        if (inliers < 20 or not np.isfinite(translation).all() or
                not np.isfinite(ratio) or ratio <= 0.0 or
                not np.isfinite(reprojection) or reprojection <= 0.0):
            stats["pnp_invalid"] += 1
            return
        depth_stamp = stamp_ns(depth_message)
        if depth_stamp > ref_stamp or ref_stamp - depth_stamp > MAX_DEPTH_AGE_NS:
            raise RuntimeError("internal_error_noncausal_or_stale_depth_selection")
        writer.writerow((pair_index, pair_index + 1, ref_stamp, cur_stamp,
                         depth_stamp, *(format(float(v), ".17g") for v in translation),
                         inliers, format(ratio, ".17g"),
                         format(reprojection, ".17g"), "VALID_CAUSAL_LIDAR_DEPTH"))
        stats["pnp_valid"] += 1

    with temp_output.open("w", newline="", encoding="utf-8") as output_stream:
        writer = csv.writer(output_stream, lineterminator="\n")
        writer.writerow(columns)
        with rosbag.Bag(str(RAW_BAG), "r") as image_bag, \
             rosbag.Bag(str(DERIVED_BAG), "r") as cloud_bag:
            image_iter = iter(image_bag.read_messages(topics=[IMAGE_TOPIC]))
            cloud_iter = iter(cloud_bag.read_messages(topics=[CLOUD_TOPIC]))
            image_item = next(image_iter, None)
            cloud_item = next(cloud_iter, None)
            while image_item is not None or cloud_item is not None:
                take_image = cloud_item is None
                if image_item is not None and cloud_item is not None:
                    take_image = stamp_ns(image_item[1]) < stamp_ns(cloud_item[1])
                if take_image:
                    message = image_item[1]
                    stamp = stamp_ns(message)
                    if stamp <= last_image_stamp:
                        raise RuntimeError("Corridor01 image header stamps are not monotonic")
                    last_image_stamp = stamp
                    stats["image_messages"] += 1
                    gray = np.ascontiguousarray(vision.rectify(message, calibration))
                    image_queue.append((stamp, gray))
                    image_item = next(image_iter, None)
                else:
                    message = cloud_item[1]
                    stamp = stamp_ns(message)
                    if stamp <= last_cloud_stamp:
                        raise RuntimeError("Corridor01 cloud header stamps are not monotonic")
                    last_cloud_stamp = latest_cloud_stamp = stamp
                    stats["cloud_messages"] += 1
                    cloud_queue.append((stamp, message))
                    cloud_item = next(cloud_iter, None)

                while len(image_queue) >= 2 and latest_cloud_stamp >= image_queue[0][0]:
                    ref = image_queue[0]
                    current = image_queue[1]
                    pair_index += 1
                    stats["pairs"] += 1
                    eligible_stamps = [entry[0] for entry in cloud_queue
                                       if entry[0] <= ref[0]]
                    if not eligible_stamps:
                        stats["depth_unsynchronized"] += 1
                    else:
                        chosen_stamp = eligible_stamps[-1]
                        chosen = next(entry[1] for entry in reversed(cloud_queue)
                                      if entry[0] == chosen_stamp)
                        if ref[0] - chosen_stamp > MAX_DEPTH_AGE_NS:
                            stats["depth_unsynchronized"] += 1
                        else:
                            encode_pair(ref, current, chosen, writer)
                    image_queue.popleft()
                    if args.max_pairs and pair_index >= args.max_pairs:
                        break
                    # Keep the newest cloud at/before the next reference; all
                    # older clouds can no longer be selected by a later image.
                    if image_queue:
                        while len(cloud_queue) >= 2 and cloud_queue[1][0] <= image_queue[0][0]:
                            cloud_queue.popleft()
                if not image_queue:
                    while len(cloud_queue) >= 2:
                        cloud_queue.popleft()
                if args.max_pairs and pair_index >= args.max_pairs:
                    break
                if stats["pairs"] % 250 == 0 and stats["pairs"]:
                    output_stream.flush()
                    print(f"CORRIDOR_VISUAL_PROGRESS pairs={stats['pairs']} valid={stats['pnp_valid']}",
                          flush=True)

            # Resolve pairs still pending after the cloud stream ends using the
            # last known past cloud only; no future cloud can be selected.
            while len(image_queue) >= 2:
                ref, current = image_queue[0], image_queue[1]
                pair_index += 1
                stats["pairs"] += 1
                eligible = [entry for entry in cloud_queue if entry[0] <= ref[0]]
                if not eligible or ref[0] - eligible[-1][0] > MAX_DEPTH_AGE_NS:
                    stats["depth_unsynchronized"] += 1
                else:
                    encode_pair(ref, current, eligible[-1][1], writer)
                image_queue.popleft()

    if stats["pnp_valid"] == 0:
        temp_output.unlink(missing_ok=True)
        raise RuntimeError("Corridor01 visual frontend produced zero valid causal metric factors")
    temp_output.replace(args.output)
    manifest = args.output.with_suffix(args.output.suffix + ".provenance.txt")
    manifest.write_text(
        "dataset=SuperLoc Corridor01\n"
        f"input_manifest_sha256={sha256(INPUT_MANIFEST)}\n"
        f"raw_bag_sha256={sha256(RAW_BAG)}\n"
        f"derived_v2_bag_sha256={sha256(DERIVED_BAG)}\n"
        f"official_intrinsics_sha256={sha256(CALIB_DIR / 'corridor01_intrinsics.yaml')}\n"
        f"official_extrinsics_sha256={sha256(CALIB_DIR / 'corridor01_extrinsics.yaml')}\n"
        f"image_topic={IMAGE_TOPIC}\ncloud_topic={CLOUD_TOPIC}\n"
        "time_basis=both sensors' header.stamp; bag record time ignored\n"
        "depth_frame=adapter points_rot_only; rotational deskew only; no second deskew\n"
        f"depth_causality=nearest_cloud_header_stamp_le_ref_image;max_age_ns={MAX_DEPTH_AGE_NS}\n"
        "transform= T_imu_camera * inverse(T_camera_cur_camera_ref) * inverse(T_imu_camera)\n"
        "payload=metric relative IMU translation only; PnP rotation discarded by EKF\n"
        "online_initialization=GT-free v2 first-50-cloud pose; GT not read\n"
        + "".join(f"{key}={value}\n" for key, value in stats.items())
        + f"output_sha256={sha256(args.output)}\n",
        encoding="utf-8",
    )
    print(f"CORRIDOR_VISUAL_COMPLETE {stats} sha256={sha256(args.output)}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
