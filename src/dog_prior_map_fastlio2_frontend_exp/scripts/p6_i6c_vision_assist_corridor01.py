#!/usr/bin/env python3
"""Timestamped Corridor01 LK/epipolar frontend; emits factors, never EKF updates.

The factor is conditioned on the current closed-loop IMU trajectory for the
relative camera pose and Jacobian. Monocular two-view geometry has no metric
translation scale, so this program intentionally cannot create an absolute
map-position measurement or enable visual fusion.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import hashlib
import math
import re
import time
from pathlib import Path

import cv2
import numpy as np


ROOT = Path("/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01")
RAW_BAG = ROOT / "raw/Long_Corridor_Rosbag/raw_data_core_2023-07-25-03-01-44.bag"
INTRINSICS = ROOT / "calibration/corridor01_intrinsics.yaml"
EXTRINSICS = ROOT / "calibration/corridor01_extrinsics.yaml"
TRAJECTORY = ROOT / "results/p6_i6c_framework/formal/trajectory_STRICT_BASELINE.csv"
OUT = ROOT / "results/p6_i6c_framework/vision"
INPUT_MANIFEST = ROOT / "results/p6_i6c_framework/input/input_manifest.txt"
IMAGE_TOPIC = "/camera_1/image_raw"
EXPECTED_RAW_SHA256 = "c9e127402463eebee25c54bc967eb377e026bbbdfc979eae853a5af134180811"
EXPECTED_INTRINSICS_SHA256 = "083ff73553f6df25734bfddc439fbda7eb7b01c8baeede2b4949e960f72fd370"
EXPECTED_EXTRINSICS_SHA256 = "59b02c1fe6103196ec46645c960f3908d092c0a4ba7d93c22762bcd61210b87d"
FB_MAX_PX = 1.5
RANSAC_TRIALS = 128
RANSAC_SINE_THRESHOLD = 0.02
MIN_TRACKS = 8
MAX_CORNERS = 600
MAX_POSE_BRACKET_S = 0.5
EXPECTED_IMAGE_COUNT = 6720


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def scalar(text: str, name: str) -> float:
    match = re.search(rf"\b{name}\s*:\s*([-+0-9.eE]+)", text)
    if not match:
        raise RuntimeError(f"missing camera calibration scalar {name}")
    return float(match.group(1))


def matrix4(text: str, name: str) -> np.ndarray:
    match = re.search(rf"\b{name}\s*:[\s\S]*?data\s*:\s*\[([^]]+)\]", text)
    if not match:
        raise RuntimeError(f"missing calibration matrix {name}")
    values = np.fromstring(match.group(1).replace("\n", " "), sep=",")
    if values.size != 16:
        raise RuntimeError(f"calibration matrix {name} has {values.size} elements")
    result = values.reshape(4, 4)
    if not np.isfinite(result).all() or not np.allclose(result[3], [0, 0, 0, 1]):
        raise RuntimeError(f"invalid calibration matrix {name}")
    return result


def q_to_r(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=float)
    norm = np.linalg.norm(q)
    if not np.isfinite(norm) or norm < 1e-12:
        raise RuntimeError("invalid trajectory quaternion")
    x, y, z, w = q / norm
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def slerp(a: np.ndarray, b: np.ndarray, u: float) -> np.ndarray:
    a, b = a / np.linalg.norm(a), b / np.linalg.norm(b)
    dot = float(a @ b)
    if dot < 0:
        b, dot = -b, -dot
    if dot > 0.9995:
        q = a + u * (b-a)
        return q / np.linalg.norm(q)
    theta = math.acos(max(-1.0, min(1.0, dot)))
    return (math.sin((1-u)*theta)*a + math.sin(u*theta)*b) / math.sin(theta)


class PoseSeries:
    def __init__(self, path: Path):
        self.stamps: list[int] = []
        self.positions: list[np.ndarray] = []
        self.quaternions: list[np.ndarray] = []
        with path.open(newline="") as stream:
            for row in csv.DictReader(stream):
                stamp = int(row["stamp_ns"])
                p = np.array([float(row[f"corrected_imu_t{x}"]) for x in "xyz"])
                q = np.array([float(row[f"corrected_imu_q{x}"]) for x in "xyzw"])
                if self.stamps and stamp <= self.stamps[-1]:
                    raise RuntimeError("filter trajectory timestamps are not strictly increasing")
                if not np.isfinite(p).all() or not np.isfinite(q).all():
                    raise RuntimeError("filter trajectory contains nonfinite pose")
                self.stamps.append(stamp)
                self.positions.append(p)
                self.quaternions.append(q)
        if len(self.stamps) < 2:
            raise RuntimeError("trajectory needs at least two timestamped poses")

    def at(self, stamp: int):
        j = bisect.bisect_left(self.stamps, stamp)
        if j < len(self.stamps) and self.stamps[j] == stamp:
            return self.positions[j].copy(), self.quaternions[j].copy()
        if j == 0 or j == len(self.stamps):
            return None
        t0, t1 = self.stamps[j-1], self.stamps[j]
        if (t1-t0)*1e-9 > MAX_POSE_BRACKET_S:
            return None
        u = (stamp-t0)/(t1-t0)
        return (self.positions[j-1] + u*(self.positions[j]-self.positions[j-1]),
                slerp(self.quaternions[j-1], self.quaternions[j], u))


def bearings(points: np.ndarray, k: np.ndarray, d: np.ndarray, xi: float) -> np.ndarray:
    normalized = cv2.undistortPoints(points.reshape(-1, 1, 2), k, d).reshape(-1, 2)
    x, y = normalized[:, 0], normalized[:, 1]
    r2 = x*x+y*y
    discriminant = 1.0 + (1.0-xi*xi)*r2
    valid = discriminant >= 0.0
    lam = np.full_like(r2, np.nan)
    lam[valid] = (xi + np.sqrt(discriminant[valid]))/(1.0+r2[valid])
    b = np.column_stack((lam*x, lam*y, lam-xi))
    norms = np.linalg.norm(b, axis=1)
    valid &= np.isfinite(norms) & (norms > 1e-12)
    b[valid] /= norms[valid, None]
    b[~valid] = np.nan
    return b


def skew(v: np.ndarray) -> np.ndarray:
    x, y, z = v
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def ransac_translation_direction(normals: np.ndarray, seed: int):
    norms = np.linalg.norm(normals, axis=1)
    usable = np.isfinite(norms) & (norms > 1e-9)
    rows = normals[usable] / norms[usable, None]
    if len(rows) < MIN_TRACKS:
        return None, np.zeros(len(normals), dtype=bool)
    rng = np.random.default_rng(seed)
    best = np.zeros(len(rows), dtype=bool)
    best_median = float("inf")
    for _ in range(RANSAC_TRIALS):
        pair = rng.choice(len(rows), size=2, replace=False)
        direction = np.cross(rows[pair[0]], rows[pair[1]])
        magnitude = np.linalg.norm(direction)
        if magnitude < 1e-8:
            continue
        direction /= magnitude
        errors = np.abs(rows @ direction)
        inliers = errors <= RANSAC_SINE_THRESHOLD
        count = int(inliers.sum())
        median = float(np.median(errors[inliers])) if count else float("inf")
        if count > int(best.sum()) or (count == int(best.sum()) and median < best_median):
            best, best_median = inliers, median
    if int(best.sum()) < MIN_TRACKS:
        return None, np.zeros(len(normals), dtype=bool)
    _, _, vt = np.linalg.svd(rows[best], full_matrices=False)
    direction = vt[-1]
    direction /= np.linalg.norm(direction)
    full_mask = np.zeros(len(normals), dtype=bool)
    full_mask[np.flatnonzero(usable)[best]] = True
    return direction, full_mask


def camera_pose(imu_pose, T_I_C: np.ndarray):
    p, q = imu_pose
    R_W_I = q_to_r(q)
    R_I_C = T_I_C[:3, :3]
    p_W_C = p + R_W_I @ T_I_C[:3, 3]
    R_W_C = R_W_I @ R_I_C
    return p_W_C, R_W_C


def linearize(b_ref: np.ndarray, b_cur: np.ndarray,
              R_cur_ref: np.ndarray, t_cur_ref: np.ndarray):
    residuals, jacobians = [], []
    for a, b in zip(b_ref, b_cur):
        v = R_cur_ref @ a
        normal = np.cross(v, b)
        residual = float(normal @ t_cur_ref)
        # For r=b_cur^T [t] R b_ref, columns are derivatives wrt raw
        # camera-relative translation and a left perturbation of R_cur_ref.
        j_t = normal
        j_theta = b @ skew(t_cur_ref) @ (-skew(v))
        residuals.append(residual)
        jacobians.append(np.concatenate((j_t, j_theta)))
    return np.asarray(residuals), np.asarray(jacobians)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--trajectory", type=Path, default=TRAJECTORY)
    parser.add_argument("--output", type=Path, default=OUT)
    args = parser.parse_args()
    for path in (RAW_BAG, INTRINSICS, EXTRINSICS, INPUT_MANIFEST, args.trajectory):
        if not path.is_file():
            raise FileNotFoundError(path)

    input_manifest = dict(line.split("=", 1) for line in
                          INPUT_MANIFEST.read_text().splitlines() if "=" in line)
    if input_manifest.get("raw_bag_sha256") != EXPECTED_RAW_SHA256 or \
       input_manifest.get("camera_intrinsics_sha256") != EXPECTED_INTRINSICS_SHA256 or \
       input_manifest.get("extrinsics_sha256") != EXPECTED_EXTRINSICS_SHA256:
        raise RuntimeError("vision frontend inputs differ from the pinned Corridor01 calibration/bag")
    if sha256(INTRINSICS) != EXPECTED_INTRINSICS_SHA256 or \
       sha256(EXTRINSICS) != EXPECTED_EXTRINSICS_SHA256:
        raise RuntimeError("camera calibration file SHA-256 mismatch")

    from cv_bridge import CvBridge
    import rosbag

    intrinsic_text = INTRINSICS.read_text()
    extrinsic_text = EXTRINSICS.read_text()
    xi = scalar(intrinsic_text, "xi")
    gamma1, gamma2 = scalar(intrinsic_text, "gamma1"), scalar(intrinsic_text, "gamma2")
    u0, v0 = scalar(intrinsic_text, "u0"), scalar(intrinsic_text, "v0")
    coeffs = np.array([scalar(intrinsic_text, key) for key in ("k1", "k2", "p1", "p2")])
    k = np.array([[gamma1, 0, u0], [0, gamma2, v0], [0, 0, 1.]], dtype=np.float64)
    T_I_C = matrix4(extrinsic_text, "rgb_camera_to_imu")
    poses = PoseSeries(args.trajectory)
    bridge = CvBridge()
    args.output.mkdir(parents=True, exist_ok=True)
    pair_path = args.output / "visual_assist_pairs.csv"
    factor_path = args.output / "visual_assist_normal_equations.csv"
    pair_file, factor_file = pair_path.open("w", newline=""), factor_path.open("w", newline="")
    pair_writer, factor_writer = csv.writer(pair_file, lineterminator="\n"), csv.writer(factor_file, lineterminator="\n")
    pair_writer.writerow(("pair_index", "reference_stamp_ns", "current_stamp_ns", "status",
                          "reference_features", "lk_forward", "fb_pass", "pose_bracketed",
                          "ransac_inliers", "inlier_ratio", "visual_translation_direction_camera_xyz",
                          "visual_direction_epipolar_sine_rms", "predicted_camera_translation_xyz_m",
                          "candidate_epipolar_residual_rms_m",
                          "candidate_epipolar_residual_p95_m", "linearization_mean_translation_xyz",
                          "linearization_mean_rotation_xyz", "processing_ms", "fusion_enabled"))
    factor_writer.writerow(("pair_index", "reference_stamp_ns", "current_stamp_ns", "inlier_count",
                            "residual_sum_squares", "gradient_6", "normal_matrix_upper_21",
                            "coordinates", "scale_observable", "factor_valid"))

    previous_stamp = None
    previous_gray = None
    pair_index = 0
    valid_pairs = 0
    total_fb = 0
    pair_times_ms = []
    with rosbag.Bag(str(RAW_BAG), "r") as bag:
        for _, message, _record_time in bag.read_messages(topics=[IMAGE_TOPIC]):
            stamp = int(message.header.stamp.secs)*1_000_000_000 + int(message.header.stamp.nsecs)
            if stamp <= 0 or (previous_stamp is not None and stamp <= previous_stamp):
                raise RuntimeError(f"nonmonotonic image header stamp at pair={pair_index}")
            started = time.perf_counter()
            gray = bridge.imgmsg_to_cv2(message, desired_encoding="mono8")
            gray = np.ascontiguousarray(gray)
            if previous_gray is None:
                previous_gray, previous_stamp = gray, stamp
                continue

            ref_stamp = previous_stamp
            ref_corners = cv2.goodFeaturesToTrack(
                previous_gray, maxCorners=MAX_CORNERS, qualityLevel=0.01,
                minDistance=8.0, blockSize=7, useHarrisDetector=False)
            features = 0 if ref_corners is None else len(ref_corners)
            status = "NO_FEATURES"
            forward_count = fb_count = inlier_count = 0
            ratio = float("nan")
            direction = np.full(3, np.nan)
            visual_fit_rms = float("nan")
            predicted_t = np.full(3, np.nan)
            rms = p95 = float("nan")
            mean_jt = np.full(3, np.nan)
            mean_jr = np.full(3, np.nan)
            bracketed = False

            if features:
                current_points, lk_status, _ = cv2.calcOpticalFlowPyrLK(
                    previous_gray, gray, ref_corners, None,
                    winSize=(21, 21), maxLevel=3,
                    criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
                if current_points is not None and lk_status is not None:
                    forward = lk_status.reshape(-1).astype(bool)
                    forward_count = int(forward.sum())
                    if forward_count:
                        back_points, back_status, _ = cv2.calcOpticalFlowPyrLK(
                            gray, previous_gray, current_points[forward], None,
                            winSize=(21, 21), maxLevel=3,
                            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
                        survivors = np.zeros(forward_count, dtype=bool)
                        if back_points is not None and back_status is not None:
                            back_ok = back_status.reshape(-1).astype(bool)
                            back_indices = np.flatnonzero(back_ok)
                            fb_error = np.linalg.norm(
                                ref_corners.reshape(-1, 2)[forward][back_indices] -
                                back_points.reshape(-1, 2)[back_indices], axis=1)
                            survivors[back_indices[fb_error <= FB_MAX_PX]] = True
                        fb_count = int(survivors.sum())
                        total_fb += fb_count
                        p_ref = ref_corners.reshape(-1, 2)[forward][survivors]
                        p_cur = current_points.reshape(-1, 2)[forward][survivors]
                        b_ref_all, b_cur_all = bearings(p_ref, k, coeffs, xi), bearings(p_cur, k, coeffs, xi)
                        bearing_valid = np.isfinite(b_ref_all).all(axis=1) & np.isfinite(b_cur_all).all(axis=1)
                        b_ref, b_cur = b_ref_all[bearing_valid], b_cur_all[bearing_valid]

                        pose_ref, pose_cur = poses.at(ref_stamp), poses.at(stamp)
                        bracketed = pose_ref is not None and pose_cur is not None
                        if bracketed and len(b_ref) >= MIN_TRACKS:
                            p_ref_c, R_ref_c = camera_pose(pose_ref, T_I_C)
                            p_cur_c, R_cur_c = camera_pose(pose_cur, T_I_C)
                            R_cur_ref = R_cur_c.T @ R_ref_c
                            t_cur_ref = R_cur_c.T @ (p_ref_c-p_cur_c)
                            predicted_t = t_cur_ref
                            normals = np.cross((R_cur_ref @ b_ref.T).T, b_cur)
                            visual_direction, inlier_mask = ransac_translation_direction(
                                normals, 60101 + pair_index)
                            inlier_count = int(inlier_mask.sum())
                            ratio = inlier_count/max(1, len(b_ref))
                            if visual_direction is not None:
                                candidate_dot = float(visual_direction @ t_cur_ref)
                                if candidate_dot < 0:
                                    visual_direction = -visual_direction
                                direction = visual_direction
                                visual_fit_rms = float(np.sqrt(np.mean(
                                    (normals[inlier_mask] @ visual_direction)**2)))
                                if np.linalg.norm(t_cur_ref) < 1e-4:
                                    status = "CANDIDATE_CAMERA_BASELINE_TOO_SMALL"
                                    visual_direction = None
                                if visual_direction is not None:
                                    residuals, jacobians = linearize(
                                        b_ref[inlier_mask], b_cur[inlier_mask], R_cur_ref, t_cur_ref)
                                    rms = float(np.sqrt(np.mean(residuals**2)))
                                    p95 = float(np.percentile(np.abs(residuals), 95))
                                    mean_j = jacobians.mean(axis=0)
                                    mean_jt, mean_jr = mean_j[:3], mean_j[3:]
                                    normal_matrix = jacobians.T @ jacobians
                                    gradient = jacobians.T @ residuals
                                    upper = [normal_matrix[i, j] for i in range(6) for j in range(i, 6)]
                                    factor_writer.writerow((pair_index, ref_stamp, stamp, inlier_count,
                                                            float(residuals @ residuals),
                                                            ";".join(f"{x:.12g}" for x in gradient),
                                                            ";".join(f"{x:.12g}" for x in upper),
                                                            "camera_relative_[translation_xyz,left_rotation_xyz]",
                                                            "NO_MONOCULAR_TWO_VIEW_SCALE", "1"))
                                    status = "GEOMETRY_FACTOR_AVAILABLE_FUSION_DISABLED"
                                    valid_pairs += 1
                            else:
                                status = "RANSAC_INSUFFICIENT_INLIERS"
                        elif not bracketed:
                            status = "NO_FILTER_POSE_BRACKET_AT_SENSOR_STAMPS"
                        else:
                            status = "INSUFFICIENT_BEARING_TRACKS"

            elapsed_ms = (time.perf_counter()-started)*1000.0
            pair_times_ms.append(elapsed_ms)
            pair_writer.writerow((pair_index, ref_stamp, stamp, status, features,
                                  forward_count, fb_count, int(bracketed), inlier_count,
                                  ratio, ";".join(f"{x:.9g}" for x in direction), visual_fit_rms,
                                  ";".join(f"{x:.9g}" for x in predicted_t), rms, p95,
                                  ";".join(f"{x:.9g}" for x in mean_jt),
                                  ";".join(f"{x:.9g}" for x in mean_jr), elapsed_ms, 0))
            pair_index += 1
            previous_gray, previous_stamp = gray, stamp
            if pair_index % 500 == 0:
                pair_file.flush(); factor_file.flush()
                print(f"VISION_PROGRESS pairs={pair_index} factors={valid_pairs}", flush=True)

    pair_file.close(); factor_file.close()
    if pair_index + 1 != EXPECTED_IMAGE_COUNT:
        raise RuntimeError(f"expected {EXPECTED_IMAGE_COUNT} image messages, got {pair_index+1}")
    metadata = [
        "PAPER-P6-I6C Corridor01 LK visual geometry frontend",
        f"raw_bag_sha256={input_manifest['raw_bag_sha256']}",
        f"trajectory_sha256={sha256(args.trajectory)}",
        f"intrinsics_sha256={sha256(INTRINSICS)}",
        f"extrinsics_sha256={sha256(EXTRINSICS)}",
        f"image_topic={IMAGE_TOPIC}",
        "time_basis=sensor_header_stamp_ns; bag record time ignored",
        "camera_pose=interpolated STRICT_BASELINE corrected IMU pose composed with official rgb_camera_to_imu extrinsic",
        "timestamp_caveat=offline bracket interpolation uses scan-end states on both sides; this is not causal online history replay and is a fusion blocker",
        "geometry=forward-backward LK + deterministic known-rotation translation-direction RANSAC + two-view epipolar residual",
        "factor_jacobian=analytic wrt raw camera-relative translation XYZ and left relative rotation XYZ",
        "translation_scale=UNOBSERVABLE monocular two-view; no metric position measurement emitted",
        "GT_used=NO",
        "fusion_enabled=NO; requires timestamped state-history chain rule and metric scale observability",
        f"image_pairs={pair_index}",
        f"geometry_factor_pairs={valid_pairs}",
        f"total_forward_backward_tracks={total_fb}",
        f"pair_processing_ms_mean={float(np.mean(pair_times_ms)):.9g}",
        f"pair_processing_ms_p95={float(np.percentile(pair_times_ms, 95)):.9g}",
        f"pair_processing_ms_max={float(np.max(pair_times_ms)):.9g}",
        f"pair_csv_sha256={sha256(pair_path)}",
        f"normal_equation_csv_sha256={sha256(factor_path)}",
    ]
    (args.output / "visual_assist_manifest.txt").write_text("\n".join(metadata)+"\n")
    print(f"VISION_COMPLETE pairs={pair_index} factors={valid_pairs} output={args.output}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
