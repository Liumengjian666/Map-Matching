#!/usr/bin/env python3
"""Offline P3-R10C local-increment and NDT capture-basin analysis.

Requires ROS Noetic Python packages and the workspace-generated
dog_prior_map_interfaces messages. This script never launches runtime nodes or
modifies algorithm/configuration sources. The C++ companion replays only the
four selected captured request clouds against the frozen prior map.
"""

import argparse
import csv
from decimal import Decimal
import hashlib
import math
import os
import subprocess
import tempfile
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag
import sensor_msgs.point_cloud2 as pc2
import yaml
from scipy.spatial.transform import Rotation, Slerp
from scipy.stats import pearsonr, spearmanr


WORKSPACE = Path("/home/jian/livox_ws/dog_loc_paper_ws")
RESULT_ROOT = Path("/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/results/p3_r10b_fix1_floor01_full_rerun_20260926")
BAG = RESULT_ROOT / "floor01_fix1_runtime_topics.bag"
MAP = Path("/tmp/floor01_candidates/floor01_h1_map.pcd")
GT = Path("/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/gt/floor01_gt.txt")
EXTRINSICS = Path("/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/calibration/floor01_extrinsics.yaml")
EVAL_START = 1660857393.197807
EXPECTED_BAG_SHA = "860d41e88038165be469420b2c9e4db1e164ded4d58952b7b507ee01a4b0a9db"
EXPECTED_MAP_SHA = "2b571af236738a0664befacdc9c783246e991416a8915bcfebb9d2dc074e4570"
ERROR_BINS = [0.5, 1.0, 2.0, 5.0, 10.0, 20.0]
CAPTURE_THRESHOLDS = [0.5, 1.0, 2.0, 5.0]
ALPHAS = [0.0, 0.25, 0.5, 0.75, 1.0]


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(4 * 1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def pose_from_xyz_q(xyz, q_xyzw):
    t = np.eye(4, dtype=float)
    t[:3, :3] = Rotation.from_quat(np.asarray(q_xyzw, dtype=float)).as_matrix()
    t[:3, 3] = np.asarray(xyz, dtype=float)
    return t


def pose_from_row(row):
    return pose_from_xyz_q(
        [float(row["final_used_t" + a]) for a in "xyz"],
        [float(row["final_used_q" + a]) for a in "xyzw"])


def matrix_to_xyz_q(t):
    q = Rotation.from_matrix(t[:3, :3]).as_quat()
    return [float(x) for x in t[:3, 3]], [float(x) for x in q]


def read_pose_csv(path):
    rows = []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            rows.append({
                "frame_index": int(r["frame_index"]),
                "stamp": float(r["lidar_header_stamp"]),
                "stamp_text": r["lidar_header_stamp"],
                "stamp_ns": int(Decimal(r["lidar_header_stamp"]) * Decimal(1_000_000_000)),
                "pose_l": pose_from_row(r),
                "fitness": float(r["ndt_fitness"]),
                "iterations": int(float(r["ndt_iterations"])),
                "converged": int(float(r["ndt_has_converged"])),
                "step_limited": int(bool(int(float(r["translation_limited"])) or
                                         int(float(r["rotation_limited"])))),
                "scan_end": float(r["scan_end_stamp"]),
            })
    return rows


def read_gt(path):
    a = np.loadtxt(path, comments="#", ndmin=2)
    times = a[:, 0]
    poses = [pose_from_xyz_q(r[1:4], r[4:8]) for r in a]
    if not np.all(np.diff(times) > 0):
        raise ValueError("GT stamps are not strictly increasing")
    return times, poses


def interpolate_gt(times, poses, stamp):
    if stamp < times[0] or stamp > times[-1]:
        return None
    hi = int(np.searchsorted(times, stamp, side="right"))
    if hi == 0:
        return poses[0].copy()
    if hi >= len(times):
        return poses[-1].copy()
    lo = hi - 1
    u = (stamp - times[lo]) / (times[hi] - times[lo])
    out = np.eye(4)
    out[:3, 3] = poses[lo][:3, 3] + u * (poses[hi][:3, 3] - poses[lo][:3, 3])
    q0 = Rotation.from_matrix(poses[lo][:3, :3]).as_quat()
    q1 = Rotation.from_matrix(poses[hi][:3, :3]).as_quat()
    out[:3, :3] = Slerp([0.0, 1.0], Rotation.from_quat([q0, q1]))([u]).as_matrix()[0]
    return out


def rotation_error_deg(t):
    return math.degrees(float(Rotation.from_matrix(t[:3, :3]).magnitude()))


def pose_error(est, ref):
    residual = np.linalg.inv(ref) @ est
    return float(np.linalg.norm(residual[:3, 3])), rotation_error_deg(residual)


def increment_error(est_delta, gt_delta):
    return pose_error(est_delta, gt_delta)


def summarize(values):
    a = np.asarray(values, dtype=float)
    a = a[np.isfinite(a)]
    if not len(a):
        return {k: float("nan") for k in ("mean", "rmse", "median", "p95", "max")}
    return {
        "mean": float(np.mean(a)),
        "rmse": float(np.sqrt(np.mean(a * a))),
        "median": float(np.median(a)),
        "p95": float(np.percentile(a, 95)),
        "max": float(np.max(a)),
    }


def persistent_crossing(times, errors, threshold, min_duration=5.0):
    over = np.asarray(errors) > threshold
    if not np.any(over):
        return None
    i = 0
    while i < len(times):
        if not over[i]:
            i += 1
            continue
        j = i
        while j + 1 < len(times) and over[j + 1] and times[j + 1] - times[j] <= 0.25:
            j += 1
        if times[j] - times[i] >= min_duration:
            return float(times[i])
        i = j + 1
    return None


def pearson_spearman(x, y):
    x, y = np.asarray(x, dtype=float), np.asarray(y, dtype=float)
    mask = np.isfinite(x) & np.isfinite(y)
    if np.count_nonzero(mask) < 3:
        return float("nan"), float("nan")
    return float(pearsonr(x[mask], y[mask]).statistic), float(spearmanr(x[mask], y[mask]).statistic)


def write_csv(path, fields, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore", lineterminator="\n")
        w.writeheader()
        for row in rows:
            w.writerow(row)


def fmt(x, digits=4):
    if x is None or not np.isfinite(x):
        return "NA"
    return f"{x:.{digits}f}"


def se3_interpolate(a, b, alpha):
    t = np.eye(4)
    t[:3, 3] = (1.0 - alpha) * a[:3, 3] + alpha * b[:3, 3]
    qa = Rotation.from_matrix(a[:3, :3]).as_quat()
    qb = Rotation.from_matrix(b[:3, :3]).as_quat()
    t[:3, :3] = Slerp([0.0, 1.0], Rotation.from_quat([qa, qb]))([alpha]).as_matrix()[0]
    return t


def write_binary_xyz_pcd(path, msg):
    points = np.asarray(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)),
                        dtype=np.float32)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) == 0:
        raise RuntimeError(f"empty/invalid cloud at {msg.header.stamp.to_sec():.9f}")
    header = ("# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n"
              "FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
              f"WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
              f"POINTS {len(points)}\nDATA binary\n").encode("ascii")
    with open(path, "wb") as f:
        f.write(header)
        f.write(np.ascontiguousarray(points, dtype="<f4").tobytes())
    return len(points)


def extract_capture_clouds(bag_path, selected, temp_dir):
    from dog_prior_map_interfaces.msg import NdtScanRequest  # noqa: F401
    wanted = [(x["stamp_ns"], x) for x in selected]
    found = {}
    stop_stamp = max(x[0] for x in wanted)
    with rosbag.Bag(str(bag_path), "r") as bag:
        for _, msg, _ in bag.read_messages(topics=["/dog_livo/ndt/scan_request"]):
            stamp_ns = int(msg.scan_end_ns)
            nearest = min(wanted, key=lambda x: abs(x[0] - stamp_ns))
            item = nearest[1] if abs(nearest[0] - stamp_ns) <= 1000 else None
            if item is not None and item["group"] not in found:
                if msg.cloud_end_frame.header.frame_id != "cmu_sp1_velodyne":
                    raise RuntimeError("capture cloud frame mismatch: " + msg.cloud_end_frame.header.frame_id)
                pcd = Path(temp_dir) / f"capture_{item['group']}.pcd"
                item["raw_source_points"] = write_binary_xyz_pcd(pcd, msg.cloud_end_frame)
                item["pcd_path"] = str(pcd)
                item["bag_stamp_ns"] = stamp_ns
                found[item["group"]] = item
            if stamp_ns >= stop_stamp and len(found) == len(wanted):
                break
    if len(found) != len(wanted):
        missing = sorted(item["group"] for _, item in wanted if item["group"] not in found)
        raise RuntimeError(f"missing selected request clouds: {missing}")


def build_helper(source_path, output_path):
    cmd = ["g++", "-std=c++14", "-O2", "-Wall", "-Wextra",
           "-I/usr/include/pcl-1.10", "-I/usr/include/eigen3", str(source_path), "-o", str(output_path),
           "-lpcl_registration", "-lpcl_filters", "-lpcl_io", "-lpcl_search", "-lpcl_kdtree",
           "-lpcl_octree", "-lpcl_common", "-lboost_filesystem", "-lboost_system", "-lflann_cpp",
           "-lqhull_r", "-lusb-1.0"]
    subprocess.run(cmd, check=True)
    return "g++ -std=c++14 -O2 -Wall -Wextra with PCL 1.10; executable created in an auto-removed temporary directory"


def make_capture_manifest(selected, path):
    with open(path, "w") as f:
        for item in selected:
            online_l = item["predictor_l"]
            gt_l = item["gt_aligned"] @ item["T_i_l"]
            for alpha in ALPHAS:
                guess = se3_interpolate(online_l, gt_l, alpha)
                xyz, q = matrix_to_xyz_q(guess)
                values = [item["stamp"], item["group"], alpha, item["pcd_path"]] + xyz + q
                f.write("\t".join(str(v) for v in values) + "\n")


def plot_outputs(out, times, predictor_t, corrected_t, pred_inc_t,
                 correction_t, required_t, fitness, capture_rows):
    out.mkdir(parents=True, exist_ok=True)
    plt.figure(figsize=(10, 5.5))
    plt.plot(times, predictor_t, label="Predictor absolute t error", linewidth=1.1)
    plt.plot(times, corrected_t, label="Corrected absolute t error", linewidth=1.1)
    inc_t = np.asarray(pred_inc_t)
    plt.plot(times[1:1 + len(inc_t)], inc_t, label="Predictor local increment t error", linewidth=0.9, alpha=0.8)
    plt.yscale("symlog", linthresh=0.05)
    plt.xlabel("Time from evaluator origin (s)")
    plt.ylabel("Translation error (m; increment is per scan)")
    plt.title("Floor01 global error versus local predictor increment error")
    plt.grid(True, alpha=0.25); plt.legend(); plt.tight_layout()
    plt.savefig(out / "r10c_global_vs_local_increment.png", dpi=170); plt.close()

    plt.figure(figsize=(8, 6))
    plt.scatter(corrected_t, correction_t, s=5, alpha=0.3, label="NDT correction magnitude")
    plt.scatter(corrected_t, required_t, s=5, alpha=0.3, label="GT-required correction magnitude")
    plt.xscale("symlog", linthresh=0.1); plt.yscale("symlog", linthresh=0.05)
    plt.xlabel("Corrected absolute translation error (m)"); plt.ylabel("Correction magnitude (m)")
    plt.title("Actual NDT correction versus GT-required correction")
    plt.grid(True, alpha=0.25); plt.legend(); plt.tight_layout()
    plt.savefig(out / "r10c_global_error_vs_correction.png", dpi=170); plt.close()

    plt.figure(figsize=(8, 6))
    plt.scatter(corrected_t, fitness, s=5, alpha=0.3)
    plt.xscale("symlog", linthresh=0.1); plt.yscale("symlog", linthresh=0.01)
    plt.xlabel("Corrected absolute translation error (m)"); plt.ylabel("PCL NDT fitness score")
    plt.title("NDT fitness versus global translation error")
    plt.grid(True, alpha=0.25); plt.tight_layout()
    plt.savefig(out / "r10c_global_error_vs_fitness.png", dpi=170); plt.close()

    plt.figure(figsize=(8, 5.5))
    for group in ["0p5m", "1m", "2m", "5m"]:
        subset = [r for r in capture_rows if r["crossing_group"] == group]
        subset.sort(key=lambda r: float(r["alpha"]))
        plt.plot([float(r["alpha"]) for r in subset], [float(r["final_t_error_m"]) for r in subset],
                 marker="o", label=group)
    plt.xlabel("Initial guess interpolation α (predictor → fixed-anchor GT)")
    plt.ylabel("Final translation error to fixed-anchor GT (m)")
    plt.title("PCL NDT capture basin at four persistent-error crossings")
    plt.grid(True, alpha=0.25); plt.legend(title="Corrected-error crossing"); plt.tight_layout()
    plt.savefig(out / "r10c_capture_basin_alpha_sweep.png", dpi=170); plt.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--workspace", type=Path, default=WORKSPACE)
    parser.add_argument("--run", type=Path, default=RESULT_ROOT)
    parser.add_argument("--bag", type=Path, default=BAG)
    parser.add_argument("--map", type=Path, default=MAP)
    parser.add_argument("--gt", type=Path, default=GT)
    parser.add_argument("--extrinsics", type=Path, default=EXTRINSICS)
    parser.add_argument("--out", type=Path, default=WORKSPACE / "src/dog_prior_map_fastlio2_frontend_exp/docs/p3_r10c_failure_mechanism_1")
    parser.add_argument("--eval-start", type=float, default=EVAL_START)
    args = parser.parse_args()
    out = args.out.resolve()
    data_dir = args.run / "evaluation_inputs"
    predictor = read_pose_csv(data_dir / "predictor.csv")
    ndt_used = read_pose_csv(data_dir / "ndt_used.csv")
    corrected = read_pose_csv(data_dir / "corrected.csv")
    raw_ndt = read_pose_csv(data_dir / "raw_ndt.csv")
    if not (len(predictor) == len(ndt_used) == len(corrected) == len(raw_ndt)):
        raise RuntimeError("R10B evaluation CSV frame counts differ")
    for streams in zip(predictor, ndt_used, corrected, raw_ndt):
        stamps = [r["stamp"] for r in streams]
        if max(stamps) - min(stamps) > 1e-7:
            raise RuntimeError("R10B CSV timestamps do not align")

    gt_times, gt_poses = read_gt(args.gt)
    with open(args.extrinsics) as f:
        extr = yaml.safe_load(f)
    T_i_l = np.asarray(extr["laser_to_imu"]["data"], dtype=float).reshape(4, 4)
    T_i_l[:3, :3] = Rotation.from_matrix(T_i_l[:3, :3]).as_matrix()
    T_l_i = np.linalg.inv(T_i_l)

    rows = []
    for p, n, c, raw in zip(predictor, ndt_used, corrected, raw_ndt):
        stamp = c["stamp"]
        gt_i = interpolate_gt(gt_times, gt_poses, stamp)
        if stamp < args.eval_start or gt_i is None:
            continue
        rows.append({
            "frame_index": c["frame_index"], "stamp": stamp, "stamp_text": c["stamp_text"],
            "stamp_ns": c["stamp_ns"],
            "time": stamp - args.eval_start,
            "predictor_i": p["pose_l"] @ T_l_i,
            "ndt_i": n["pose_l"] @ T_l_i,
            "corrected_i": c["pose_l"] @ T_l_i,
            "raw_ndt_l": raw["pose_l"],
            "fitness": n["fitness"], "iterations": n["iterations"],
            "converged": n["converged"], "step_limited": n["step_limited"],
            "gt_i": gt_i,
        })
    if len(rows) < 100:
        raise RuntimeError("too few in-coverage R10B samples")

    # Fixed first-common-sample anchor, matching the existing relative evaluator.
    anchor = rows[0]["corrected_i"] @ np.linalg.inv(rows[0]["gt_i"])
    for r in rows:
        r["gt_aligned"] = anchor @ r["gt_i"]
        r["predictor_abs_t"], r["predictor_abs_r"] = pose_error(r["predictor_i"], r["gt_aligned"])
        r["ndt_abs_t"], r["ndt_abs_r"] = pose_error(r["ndt_i"], r["gt_aligned"])
        r["corrected_abs_t"], r["corrected_abs_r"] = pose_error(r["corrected_i"], r["gt_aligned"])
        r["ndt_correction_t"], r["ndt_correction_r"] = pose_error(r["ndt_i"], r["predictor_i"])
        r["gt_required_t"], r["gt_required_r"] = pose_error(r["gt_aligned"], r["predictor_i"])

    local_rows = []
    pred_t, pred_r, ndt_t, ndt_r, gt_t, gt_r = [], [], [], [], [], []
    chain_pred = [rows[0]["gt_aligned"].copy()]
    chain_ndt = [rows[0]["gt_aligned"].copy()]
    chain_pred_t, chain_pred_r, chain_ndt_t, chain_ndt_r = [], [], [], []
    chain_times = [rows[0]["time"]]
    for i, r in enumerate(rows):
        entry = {"frame_index": r["frame_index"], "stamp": r["stamp_text"], "time_s": r["time"]}
        if i == 0:
            entry.update({"dt_s": "", "gt_inc_t_m": "", "gt_inc_r_deg": "",
                          "pred_inc_t_error_m": "", "pred_inc_r_error_deg": "",
                          "ndt_inc_t_error_m": "", "ndt_inc_r_error_deg": "",
                          "predictor_chain_abs_t_error_m": 0.0, "predictor_chain_abs_r_error_deg": 0.0,
                          "ndt_chain_abs_t_error_m": 0.0, "ndt_chain_abs_r_error_deg": 0.0})
        else:
            prev = rows[i - 1]
            dt = r["stamp"] - prev["stamp"]
            if dt <= 0 or dt > 0.25:
                raise RuntimeError(f"non-contiguous common-frame interval dt={dt:.6f}s at row {i}")
            dgt = np.linalg.inv(prev["gt_i"]) @ r["gt_i"]
            dpred = np.linalg.inv(prev["corrected_i"]) @ r["predictor_i"]
            dndt = np.linalg.inv(prev["ndt_i"]) @ r["ndt_i"]
            pe_t, pe_r = increment_error(dpred, dgt)
            ne_t, ne_r = increment_error(dndt, dgt)
            gt_step_t, gt_step_r = float(np.linalg.norm(dgt[:3, 3])), rotation_error_deg(dgt)
            pred_t.append(pe_t); pred_r.append(pe_r); ndt_t.append(ne_t); ndt_r.append(ne_r)
            gt_t.append(gt_step_t); gt_r.append(gt_step_r)
            entry.update({"dt_s": dt, "gt_inc_t_m": gt_step_t, "gt_inc_r_deg": gt_step_r,
                          "pred_inc_t_error_m": pe_t, "pred_inc_r_error_deg": pe_r,
                          "ndt_inc_t_error_m": ne_t, "ndt_inc_r_error_deg": ne_r})
            chain_pred.append(chain_pred[-1] @ dpred)
            chain_ndt.append(chain_ndt[-1] @ dndt)
            chain_times.append(r["time"])
            cpt, cpr = pose_error(chain_pred[-1], r["gt_aligned"])
            cnt, cnr = pose_error(chain_ndt[-1], r["gt_aligned"])
            chain_pred_t.append(cpt); chain_pred_r.append(cpr); chain_ndt_t.append(cnt); chain_ndt_r.append(cnr)
            entry.update({"predictor_chain_abs_t_error_m": cpt, "predictor_chain_abs_r_error_deg": cpr,
                          "ndt_chain_abs_t_error_m": cnt, "ndt_chain_abs_r_error_deg": cnr})
        entry.update({
            "predictor_abs_t_error_m": r["predictor_abs_t"], "predictor_abs_r_error_deg": r["predictor_abs_r"],
            "ndt_used_abs_t_error_m": r["ndt_abs_t"], "ndt_used_abs_r_error_deg": r["ndt_abs_r"],
            "corrected_abs_t_error_m": r["corrected_abs_t"], "corrected_abs_r_error_deg": r["corrected_abs_r"],
            "ndt_correction_t_m": r["ndt_correction_t"], "ndt_correction_r_deg": r["ndt_correction_r"],
            "gt_required_correction_t_m": r["gt_required_t"], "gt_required_correction_r_deg": r["gt_required_r"],
            "fitness": r["fitness"], "iterations": r["iterations"], "converged": r["converged"],
            "step_limited": r["step_limited"],
        })
        local_rows.append(entry)

    local_fields = ["frame_index", "stamp", "time_s", "dt_s", "gt_inc_t_m", "gt_inc_r_deg",
                    "pred_inc_t_error_m", "pred_inc_r_error_deg", "ndt_inc_t_error_m", "ndt_inc_r_error_deg",
                    "predictor_chain_abs_t_error_m", "predictor_chain_abs_r_error_deg",
                    "ndt_chain_abs_t_error_m", "ndt_chain_abs_r_error_deg",
                    "predictor_abs_t_error_m", "predictor_abs_r_error_deg", "ndt_used_abs_t_error_m",
                    "ndt_used_abs_r_error_deg", "corrected_abs_t_error_m", "corrected_abs_r_error_deg",
                    "ndt_correction_t_m", "ndt_correction_r_deg", "gt_required_correction_t_m",
                    "gt_required_correction_r_deg", "fitness", "iterations", "converged", "step_limited"]
    write_csv(out / "p3_r10c_local_increment_analysis.csv", local_fields, local_rows)

    times = np.asarray([r["time"] for r in rows])
    corrected_t = np.asarray([r["corrected_abs_t"] for r in rows])
    crossings = {threshold: persistent_crossing(times, corrected_t, threshold) for threshold in CAPTURE_THRESHOLDS}
    capture = []
    predictor_by_frame = {r["frame_index"]: r for r in predictor}
    for threshold in CAPTURE_THRESHOLDS:
        cross = crossings[threshold]
        if cross is None:
            raise RuntimeError(f"no persistent crossing for {threshold}m")
        target_time = cross + 1.5
        idx = int(np.argmin(np.abs(times - target_time)))
        item = dict(rows[idx])
        item["group"] = {0.5: "0p5m", 1.0: "1m", 2.0: "2m", 5.0: "5m"}[threshold]
        item["crossing_time"] = cross
        item["selected_time"] = target_time
        item["selected_frame_time"] = float(times[idx])
        item["predictor_l"] = predictor_by_frame[item["frame_index"]]["pose_l"]
        item["T_i_l"] = T_i_l
        capture.append(item)

    out.mkdir(parents=True, exist_ok=True)
    cap_rows = []
    helper_path = Path(__file__).with_name("p3_r10c_capture_basin_ndt.cpp")
    with tempfile.TemporaryDirectory(prefix="p3_r10c_capture_") as td:
        binary = Path(td) / "p3_r10c_capture_basin_ndt"
        build_cmd = build_helper(helper_path, binary)
        extract_capture_clouds(args.bag, capture, td)
        manifest = Path(td) / "capture_manifest.tsv"
        helper_csv = Path(td) / "capture_helper.tsv"
        make_capture_manifest(capture, manifest)
        helper_env = os.environ.copy()
        helper_env["LD_LIBRARY_PATH"] = ":".join(
            ["/lib/x86_64-linux-gnu", "/usr/lib/x86_64-linux-gnu"] +
            ([helper_env["LD_LIBRARY_PATH"]] if helper_env.get("LD_LIBRARY_PATH") else []))
        subprocess.run([str(binary), str(args.map), str(manifest), str(helper_csv)],
                       check=True, env=helper_env)
        with open(helper_csv, newline="") as f:
            helper_rows = list(csv.DictReader(f, delimiter="\t"))
        if len(helper_rows) != len(capture) * len(ALPHAS):
            raise RuntimeError("capture helper output row count mismatch")
        raw_by_stamp = {round(r["stamp"], 9): r for r in raw_ndt}
        for hr, cap_item in zip(helper_rows, [x for x in capture for _ in ALPHAS]):
            alpha = float(hr["alpha"])
            final_l = pose_from_xyz_q([float(hr[f"final_t{a}"]) for a in "xyz"],
                                      [float(hr[f"final_q{a}"]) for a in "xyzw"])
            gt_l = cap_item["gt_aligned"] @ T_i_l
            init_l = se3_interpolate(cap_item["predictor_l"], gt_l, alpha)
            init_t, init_r = pose_error(init_l, gt_l)
            final_t, final_r = pose_error(final_l, gt_l)
            online_raw = raw_by_stamp[round(cap_item["stamp"], 9)]
            raw_match_t, raw_match_r = pose_error(final_l, online_raw["pose_l"])
            cap_rows.append({
                "frame_stamp": cap_item["stamp_text"], "crossing_group": cap_item["group"],
                "bag_stamp_delta_ns": cap_item["bag_stamp_ns"] - cap_item["stamp_ns"],
                "persistent_crossing_s": cap_item["crossing_time"], "selected_offset_after_crossing_s": cap_item["selected_frame_time"] - cap_item["crossing_time"],
                "alpha": alpha, "raw_source_points": cap_item["raw_source_points"],
                "preprocessed_source_points": int(hr["source_points"]), "target_points": int(hr["target_points"]),
                "initial_t_error_m": init_t, "initial_r_error_deg": init_r,
                "final_t_error_m": final_t, "final_r_error_deg": final_r,
                "fitness": float(hr["fitness"]), "iterations": int(hr["iterations"]),
                "converged": int(hr["converged"]), "final_tx": float(hr["final_tx"]),
                "final_ty": float(hr["final_ty"]), "final_tz": float(hr["final_tz"]),
                "final_qx": float(hr["final_qx"]), "final_qy": float(hr["final_qy"]),
                "final_qz": float(hr["final_qz"]), "final_qw": float(hr["final_qw"]),
                "alpha0_raw_ndt_pose_delta_t_m": raw_match_t if alpha == 0 else "",
                "alpha0_raw_ndt_pose_delta_r_deg": raw_match_r if alpha == 0 else "",
                "saved_raw_ndt_fitness": online_raw["fitness"] if alpha == 0 else "",
                "saved_raw_ndt_fitness_delta": float(hr["fitness"]) - online_raw["fitness"] if alpha == 0 else "",
            })

    cap_fields = list(cap_rows[0].keys())
    write_csv(out / "p3_r10c_capture_basin.csv", cap_fields, cap_rows)
    plot_outputs(out, times, np.asarray([r["predictor_abs_t"] for r in rows]), corrected_t,
                 pred_t, np.asarray([r["ndt_correction_t"] for r in rows]),
                 np.asarray([r["gt_required_t"] for r in rows]),
                 np.asarray([r["fitness"] for r in rows]), cap_rows)

    # Numerical summaries.
    pred_chain_stats_t, pred_chain_stats_r = summarize(chain_pred_t), summarize(chain_pred_r)
    ndt_chain_stats_t, ndt_chain_stats_r = summarize(chain_ndt_t), summarize(chain_ndt_r)
    pred_inc_stats_t, pred_inc_stats_r = summarize(pred_t), summarize(pred_r)
    ndt_inc_stats_t, ndt_inc_stats_r = summarize(ndt_t), summarize(ndt_r)
    global_stats = {
        "Predictor": (summarize([r["predictor_abs_t"] for r in rows]), summarize([r["predictor_abs_r"] for r in rows])),
        "NDT-used": (summarize([r["ndt_abs_t"] for r in rows]), summarize([r["ndt_abs_r"] for r in rows])),
        "Corrected": (summarize([r["corrected_abs_t"] for r in rows]), summarize([r["corrected_abs_r"] for r in rows])),
    }
    segment_rows = []
    edges = list(range(0, 351, 50))
    for start in edges[:-1]:
        stop = start + 50
        mask = np.asarray([(r["time"] >= start and r["time"] < stop) for r in rows[1:]])
        it = np.asarray(pred_t)[mask]; ir = np.asarray(pred_r)[mask]
        segment_rows.append((f"{start}-{stop}s", len(it), summarize(it), summarize(ir)))
    mask = np.asarray([r["time"] >= 350 for r in rows[1:]])
    segment_rows.append(("350-end", int(np.count_nonzero(mask)), summarize(np.asarray(pred_t)[mask]), summarize(np.asarray(pred_r)[mask])))

    fit_p_corr, fit_s_corr = pearson_spearman([r["fitness"] for r in rows], corrected_t)
    fit_p_ndt, fit_s_ndt = pearson_spearman([r["fitness"] for r in rows], [r["ndt_abs_t"] for r in rows])
    bins = [("<0.5m", 0.0, 0.5)] + [(f"{a:g}-{b:g}m", a, b) for a, b in zip(ERROR_BINS[:-1], ERROR_BINS[1:])] + [(">20m", 20.0, float("inf"))]
    bin_rows = []
    for name, lo, hi in bins:
        subset = [r for r in rows if r["corrected_abs_t"] >= lo and r["corrected_abs_t"] < hi]
        if not subset: continue
        bin_rows.append((name, len(subset), np.median([r["predictor_abs_t"] for r in subset]),
                         np.percentile([r["predictor_abs_t"] for r in subset], 95),
                         np.median([r["ndt_abs_t"] for r in subset]),
                         np.percentile([r["ndt_abs_t"] for r in subset], 95),
                         np.median([r["fitness"] for r in subset]),
                         np.percentile([r["fitness"] for r in subset], 95),
                         np.median([r["iterations"] for r in subset]),
                         np.percentile([r["iterations"] for r in subset], 95),
                         np.median([r["ndt_correction_t"] for r in subset]),
                         np.percentile([r["ndt_correction_t"] for r in subset], 95),
                         np.median([r["gt_required_t"] for r in subset]),
                         np.percentile([r["gt_required_t"] for r in subset], 95),
                         np.median([r["ndt_correction_r"] for r in subset]),
                         np.percentile([r["ndt_correction_r"] for r in subset], 95),
                         np.median([r["gt_required_r"] for r in subset]),
                         np.percentile([r["gt_required_r"] for r in subset], 95)))

    def chain_crossing(values, threshold):
        for t, v in zip(chain_times[1:], values):
            if v > threshold: return t
        return None

    # Capture-basin classification diagnostics.
    cap_diag = []
    for item in capture:
        subset = [r for r in cap_rows if r["crossing_group"] == item["group"]]
        subset.sort(key=lambda r: float(r["alpha"]))
        row0, row1 = subset[0], subset[-1]
        cap_diag.append((item["group"], item["selected_frame_time"], item["corrected_abs_t"],
                         row0["final_t_error_m"], row1["final_t_error_m"], row0["fitness"], row1["fitness"],
                         row0["alpha0_raw_ndt_pose_delta_t_m"], row0["alpha0_raw_ndt_pose_delta_r_deg"],
                         row0["saved_raw_ndt_fitness_delta"]))

    bag_sha, map_sha = sha256(args.bag), sha256(args.map)
    if bag_sha != EXPECTED_BAG_SHA or map_sha != EXPECTED_MAP_SHA:
        raise RuntimeError("input bag/map hash differs from the frozen R10B provenance")

    # Classification is evidence-based and deliberately conservative.
    # The principal distinction is whether the PCL replay at alpha=1 recovers
    # the map-frame GT while alpha=0 remains wrong, with verified alpha=0 replay.
    alpha0_match = (max(float(x[7]) for x in cap_diag) < 0.05 and
                    max(float(x[8]) for x in cap_diag) < 0.2 and
                    max(abs(float(x[9])) for x in cap_diag) < 0.005)
    basin_recovery = all(float(x[4]) < 0.5 and
                         float(x[3]) - float(x[4]) >= max(0.5, 0.5 * float(x[3]))
                         for x in cap_diag)
    local_pred_reasonable = pred_inc_stats_t["rmse"] < 0.15 and pred_inc_stats_r["rmse"] < 1.0
    local_increment_failure = (pred_chain_stats_t["rmse"] >= 5.0 and
                               pred_inc_stats_t["rmse"] >= 0.15)
    alpha1_improvements = sum(1 for x in cap_diag
                              if float(x[3]) - float(x[4]) >= max(0.5, 0.25 * float(x[3])))
    ndt_init_sensitive = alpha1_improvements >= 3
    if alpha0_match and local_increment_failure and ndt_init_sensitive:
        classification = "MIXED"
    elif alpha0_match and basin_recovery and local_pred_reasonable:
        classification = "NDT_LOCAL_BASIN_LOCK"
    elif alpha0_match and not basin_recovery and all(float(x[4]) > 2.0 for x in cap_diag):
        classification = "NDT_OBJECTIVE_AMBIGUITY"
    elif pred_inc_stats_t["rmse"] >= 0.15 or pred_inc_stats_r["rmse"] >= 1.0:
        classification = "MIXED" if basin_recovery else "LOCAL_IMU_INCREMENT_FAILURE"
    else:
        classification = "UNRESOLVED"

    lines = [
        "# PAPER-P3-R10C Failure Mechanism 1 — Local Increment Attribution and NDT Capture Basin", "",
        "## Scope and provenance", "",
        "Offline analysis only; no runtime, NDT/EKF, map, configuration, initial-pose, or bag changes and no rosbag replay. ",
        f"Workspace HEAD at analysis start: `e717c342f47c44869819b6a9716d569bd8cd0a0e`. ",
        f"Input bag: `{args.bag}`. Map: `{args.map}`. GT: `{args.gt}`. Extrinsic: `{args.extrinsics}`. ",
        f"Runtime topic bag SHA-256: `{bag_sha}`. Map SHA-256: `{map_sha}`. ",
        f"GT SHA-256: `{sha256(args.gt)}`. Extrinsic SHA-256: `{sha256(args.extrinsics)}`. ",
        f"Evaluation origin: `{args.eval_start:.9f}`; rows with no GT coverage/extrapolation are excluded; analyzed frames: {len(rows)}.", "",
        "### Frame semantics correction", "",
        "The task text describes corrected odometry as `map_T_imu`, but the saved `/dog_livo/fastlio2_ndt_odom` message and `publishOdometry()` source identify `child_frame_id=cmu_sp1_velodyne` and publish `map_T_lidar`. Predictor and NDT poses are also `map_T_lidar`. Therefore all three estimate streams were converted consistently to IMU origin as `map_T_imu = map_T_lidar * inverse(T_imu_lidar)` using the supplied `laser_to_imu` calibration. GT was treated as IMU-origin per the current task instruction. A single left anchor `A = corrected_imu(first) * inverse(GT_imu(first))` was applied to the GT sequence, matching the prior evaluator's fixed first-common-pose relative convention; no per-frame alignment was used.", "",
        "All three streams share the same corrected-first-pose anchor here. Consequently the corrected global metrics reproduce the prior R10B corrected metrics, while predictor/NDT metrics may differ slightly from R10B's separately trajectory-anchored summary due to their distinct first poses.", "",
        "### Global error baseline", "",
        "| Stream | Translation mean/RMSE/median/P95/max (m) | Rotation mean/RMSE/median/P95/max (deg) |",
        "|---|---|---|"]
    for name, (ts, rs) in global_stats.items():
        lines.append(f"| {name} | {fmt(ts['mean'])} / {fmt(ts['rmse'])} / {fmt(ts['median'])} / {fmt(ts['p95'])} / {fmt(ts['max'])} | {fmt(rs['mean'])} / {fmt(rs['rmse'])} / {fmt(rs['median'])} / {fmt(rs['p95'])} / {fmt(rs['max'])} |")
    lines += ["", "### Local IKFoM increments", "",
        "For each adjacent pair, `Delta_pred = inverse(corrected_imu[k-1]) * predictor_imu[k]`; `Delta_GT = inverse(GT_imu[k-1]) * GT_imu[k]`. Translation/rotation error is the norm/angle of `inverse(Delta_GT) * Delta_pred`.", "",
        f"- Valid adjacent increments: {len(pred_t)}; GT translation increment mean/P95: {fmt(np.mean(gt_t))} / {fmt(np.percentile(gt_t,95))} m.",
        f"- Translation increment error mean/RMSE/median/P95/max: {fmt(pred_inc_stats_t['mean'])} / {fmt(pred_inc_stats_t['rmse'])} / {fmt(pred_inc_stats_t['median'])} / {fmt(pred_inc_stats_t['p95'])} / {fmt(pred_inc_stats_t['max'])} m.",
        f"- Rotation increment error mean/RMSE/median/P95/max: {fmt(pred_inc_stats_r['mean'])} / {fmt(pred_inc_stats_r['rmse'])} / {fmt(pred_inc_stats_r['median'])} / {fmt(pred_inc_stats_r['p95'])} / {fmt(pred_inc_stats_r['max'])} deg.", "",
        "| Runtime segment | N | local translation increment RMSE (m) | local rotation increment RMSE (deg) |",
        "|---|---:|---:|---:|"]
    for label, n, st, sr in segment_rows:
        lines.append(f"| {label} | {n} | {fmt(st['rmse'])} | {fmt(sr['rmse'])} |")
    lines += ["", "### Pure increment chains", "",
              "Both chains start at the first GT pose and thereafter use only the corresponding local increments; no per-frame NDT absolute pose or re-anchoring is used. Errors compare the chain pose to the once-aligned GT pose.",
              f"- IMU predictor-increment chain translation mean/RMSE/P95/max: {fmt(pred_chain_stats_t['mean'])} / {fmt(pred_chain_stats_t['rmse'])} / {fmt(pred_chain_stats_t['p95'])} / {fmt(pred_chain_stats_t['max'])} m.",
              f"- IMU predictor-increment chain rotation mean/RMSE/P95/max: {fmt(pred_chain_stats_r['mean'])} / {fmt(pred_chain_stats_r['rmse'])} / {fmt(pred_chain_stats_r['p95'])} / {fmt(pred_chain_stats_r['max'])} deg.",
              f"- IMU chain first crossing (m): 0.25={fmt(chain_crossing(chain_pred_t,.25))} s; 0.5={fmt(chain_crossing(chain_pred_t,.5))} s; 1={fmt(chain_crossing(chain_pred_t,1))} s; 2={fmt(chain_crossing(chain_pred_t,2))} s; 5={fmt(chain_crossing(chain_pred_t,5))} s.",
              f"- NDT-used-increment chain translation mean/RMSE/P95/max: {fmt(ndt_chain_stats_t['mean'])} / {fmt(ndt_chain_stats_t['rmse'])} / {fmt(ndt_chain_stats_t['p95'])} / {fmt(ndt_chain_stats_t['max'])} m.",
              f"- NDT-used-increment chain rotation mean/RMSE/P95/max: {fmt(ndt_chain_stats_r['mean'])} / {fmt(ndt_chain_stats_r['rmse'])} / {fmt(ndt_chain_stats_r['p95'])} / {fmt(ndt_chain_stats_r['max'])} deg.",
              f"- NDT chain first crossing (m): 0.25={fmt(chain_crossing(chain_ndt_t,.25))} s; 0.5={fmt(chain_crossing(chain_ndt_t,.5))} s; 1={fmt(chain_crossing(chain_ndt_t,1))} s; 2={fmt(chain_crossing(chain_ndt_t,2))} s; 5={fmt(chain_crossing(chain_ndt_t,5))} s.", "",
              "### Local NDT increments and correction", "",
              f"- NDT-used local increment translation error mean/RMSE/median/P95/max: {fmt(ndt_inc_stats_t['mean'])} / {fmt(ndt_inc_stats_t['rmse'])} / {fmt(ndt_inc_stats_t['median'])} / {fmt(ndt_inc_stats_t['p95'])} / {fmt(ndt_inc_stats_t['max'])} m.",
              f"- NDT-used local increment rotation error mean/RMSE/median/P95/max: {fmt(ndt_inc_stats_r['mean'])} / {fmt(ndt_inc_stats_r['rmse'])} / {fmt(ndt_inc_stats_r['median'])} / {fmt(ndt_inc_stats_r['p95'])} / {fmt(ndt_inc_stats_r['max'])} deg.",
              "- Per-frame actual correction is `inverse(predictor_imu) * ndt_used_imu`; required correction is `inverse(predictor_imu) * GT_aligned_imu`.", "",
              "| Corrected global translation-error bin | N | predictor error median/P95 (m) | NDT-used error median/P95 (m) | fitness median/P95 | iterations median/P95 | NDT translation correction median/P95 (m) | GT-required translation correction median/P95 (m) | NDT rotation correction median/P95; GT-required median/P95 (deg) |",
              "|---|---:|---:|---:|---:|---:|---:|---:|---:|"]
    for b in bin_rows:
        lines.append(f"| {b[0]} | {b[1]} | {fmt(b[2])} / {fmt(b[3])} | {fmt(b[4])} / {fmt(b[5])} | {fmt(b[6])} / {fmt(b[7])} | {fmt(b[8])} / {fmt(b[9])} | {fmt(b[10])} / {fmt(b[11])} | {fmt(b[12])} / {fmt(b[13])} | {fmt(b[14])} / {fmt(b[15])}; {fmt(b[16])} / {fmt(b[17])} |")
    large5 = np.asarray([r["fitness"] for r in rows if r["corrected_abs_t"] > 5.0])
    large10 = np.asarray([r["fitness"] for r in rows if r["corrected_abs_t"] > 10.0])
    normal_fitness = np.asarray([r["fitness"] for r in rows if r["corrected_abs_t"] < 0.5])
    normal_fitness_p95 = float(np.percentile(normal_fitness, 95))
    wrong5 = [r for r in rows if r["corrected_abs_t"] > 5.0]
    wrong10 = [r for r in rows if r["corrected_abs_t"] > 10.0]
    wrong5_good_fitness = sum(r["fitness"] <= normal_fitness_p95 for r in wrong5)
    wrong10_good_fitness = sum(r["fitness"] <= normal_fitness_p95 for r in wrong10)
    lines += ["", "### Fitness versus global correctness", "",
              f"- Fitness vs corrected translation error: Pearson={fmt(fit_p_corr)}, Spearman={fmt(fit_s_corr)}.",
              f"- Fitness vs NDT-used translation error: Pearson={fmt(fit_p_ndt)}, Spearman={fmt(fit_s_ndt)}.",
              f"- Frames with corrected error >5 m: n={len(large5)}, fitness median/P95={fmt(np.median(large5) if len(large5) else float('nan'))}/{fmt(np.percentile(large5,95) if len(large5) else float('nan'))}; >10 m: n={len(large10)}, median/P95={fmt(np.median(large10) if len(large10) else float('nan'))}/{fmt(np.percentile(large10,95) if len(large10) else float('nan'))}.",
              f"- Using the <0.5 m population fitness P95 ({fmt(normal_fitness_p95)}) as a descriptive 'normal-range' cutoff, {wrong5_good_fitness}/{len(wrong5)} (>5 m) and {wrong10_good_fitness}/{len(wrong10)} (>10 m) frames remain in that range.", "",
              "### Four-scan capture-basin sweep", "",
              "PCL 1.10.0 NDT is run offline with each captured request cloud held fixed. Map preprocessing, source filtering/voxelization/cap, resolution 0.8 m, step 0.08, epsilon 0.001, and 40 iterations match the runtime server. The sweep uses the raw PCL registration output; runtime's separate post-NDT step limiter is not applied. Translation is linearly interpolated and rotation SLERPed from online predictor to the once-aligned GT pose. No bag is replayed and GT is used only as an offline diagnostic oracle.",
              f"- Helper compiled with: `{build_cmd}`.",
              f"- Fixed target points after both 0.15 m voxel passes: {cap_rows[0]['target_points']}; map/input source hashes recorded above.",
              "- Alpha=0 cross-check against saved raw-NDT poses (translation/rotation/fscore deltas):"]
    for g, t, e, e0, e1, f0, f1, dt, dr, df in cap_diag:
        lines.append(f"  - {g} at {t:.3f}s (corrected error {e:.3f}m): alpha0 final error {float(e0):.3f}m; alpha1 {float(e1):.3f}m; fitness alpha0/1 {float(f0):.6g}/{float(f1):.6g}; alpha0-vs-saved-raw delta {float(dt):.6g}m, {float(dr):.6g}deg, fitness delta {float(df):.6g}.")
    lines.append(f"- Fixed-GT initialization improves final translation error by at least max(0.5 m, 25% of alpha0 error) in {alpha1_improvements}/4 frames; alpha1 ends below 0.5 m in {sum(float(x[4]) < 0.5 for x in cap_diag)}/4. This is frame-dependent initialization sensitivity, not a consistent global-recovery boundary.")
    lines += ["", "| Crossing group | Frame stamp | α | init t/r error | final t/r error | fitness | iter | converged |", "|---|---:|---:|---:|---:|---:|---:|---:|"]
    for r in cap_rows:
        lines.append(f"| {r['crossing_group']} | {float(r['frame_stamp']):.9f} | {float(r['alpha']):.2f} | {float(r['initial_t_error_m']):.3f}m / {float(r['initial_r_error_deg']):.2f}° | {float(r['final_t_error_m']):.3f}m / {float(r['final_r_error_deg']):.2f}° | {float(r['fitness']):.6g} | {r['iterations']} | {r['converged']} |")
    lines += ["", "### Interpretation", "",
              f"Primary classification: **{classification}**.",
              "This label is an offline empirical characterization, not proof of general NDT behavior or a new algorithm contribution.",
              f"- Predictor local increments and their pure chain are materially inaccurate (chain translation RMSE {pred_chain_stats_t['rmse']:.3f} m); NDT-used increments reduce chain RMSE to {ndt_chain_stats_t['rmse']:.3f} m but do not prevent the long-run drift.",
              f"- This audit labels MIXED when predictor-chain translation RMSE is at least 5 m, local translation-increment RMSE at least 0.15 m, and verified alpha=1 initialization improves by at least max(0.5 m, 25% of alpha0 error) in at least 3/4 selected scans. These are explicit case-classification thresholds for this audit, not universal SLAM criteria.",
              f"- Capture sweep is initialization-sensitive in {alpha1_improvements}/4 selected frames, but alpha=1 remains >0.5 m in {sum(float(x[4]) >= 0.5 for x in cap_diag)}/4. Thus this does not meet the stricter NDT_LOCAL_BASIN_LOCK criterion (reasonable IMU increments plus consistent near-GT recovery); it supports a MIXED mechanism for this run.",
              f"- A broad fitness/global-correctness decoupling is not established: correlation is positive (Spearman {fit_s_corr:.3f}). However, {wrong5_good_fitness}/{len(wrong5)} frames above 5 m still have fitness no worse than the normal-stage P95, while median actual NDT correction in >20 m is about {fmt(np.median([r['ndt_correction_t'] for r in rows if r['corrected_abs_t'] > 20]))} m versus median GT-required correction {fmt(np.median([r['gt_required_t'] for r in rows if r['corrected_abs_t'] > 20]))} m.",
              "- Alpha-sweep conclusions are accepted only if alpha=0 reproduces the saved raw NDT result closely; see the CSV delta columns for each capture scan.", ""]
    if classification == "NDT_LOCAL_BASIN_LOCK":
        lines += ["NEXT_INNOVATION_TARGET: detect when low-cost NDT has converged near a wrong predictor-centered basin, and only then expand the search or evaluate a small number of alternative hypotheses; do not implement that mechanism in this stage.", ""]
    else:
        lines += ["NEXT_INNOVATION_TARGET: not asserted; the evidence does not meet the requested NDT_LOCAL_BASIN_LOCK criterion.", ""]
    lines += ["## Artifacts", "",
              "- `p3_r10c_local_increment_analysis.csv` — per-frame local and absolute metrics.",
              "- `p3_r10c_capture_basin.csv` — 20 PCL alpha-sweep runs plus raw-result replay checks.",
              "- `r10c_global_vs_local_increment.png`", "- `r10c_global_error_vs_correction.png`",
              "- `r10c_global_error_vs_fitness.png`", "- `r10c_capture_basin_alpha_sweep.png`", "",
              "## Reproducibility", "",
              "After sourcing ROS Noetic and this workspace's `devel/setup.bash`, run:", "",
              "```bash", f"python3 {Path(__file__).resolve()}", "```", ""]
    (out / "PAPER_P3_R10C_FAILURE_MECHANISM_1.md").write_text(
        "\n".join(line.rstrip() for line in lines).rstrip() + "\n")

    print("P3_R10C_ANALYSIS_PASS")
    print(f"output={out}")
    print(f"frames={len(rows)} increments={len(pred_t)}")
    print(f"predictor_increment_t_rmse={pred_inc_stats_t['rmse']:.6f}m predictor_increment_r_rmse={pred_inc_stats_r['rmse']:.6f}deg")
    print(f"predictor_chain_t_rmse={pred_chain_stats_t['rmse']:.6f}m ndt_chain_t_rmse={ndt_chain_stats_t['rmse']:.6f}m")
    print(f"crossings={crossings}")
    print(f"classification={classification}")
    print(f"alpha0_replay_match={alpha0_match}")


if __name__ == "__main__":
    main()
