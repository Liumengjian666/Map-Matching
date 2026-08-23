#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从 Livox 原始 rosbag + 建图位姿文件生成先验关键帧扫描库。

输出：
1. keyframe_poses.txt：每行 stamp x y z qx qy qz qw。
2. scans/key_000000.pcd ...：对应关键帧的雷达原始单帧点云，坐标仍在雷达/body系。

这个工具只离线运行一次；在线定位节点读取生成结果，不改变主 NDT 定位流程。
"""

import argparse
import math
from pathlib import Path

import numpy as np
import rosbag


def load_poses(path):
    rows = []
    for line in Path(path).read_text().splitlines():
        parts = line.split()
        if len(parts) < 8:
            continue
        vals = [float(v) for v in parts[:8]]
        rows.append(vals)
    if not rows:
        raise RuntimeError(f"pose file is empty: {path}")
    return np.asarray(rows, dtype=float)


def select_keyposes(poses, spacing):
    selected = []
    last_xyz = None
    for row in poses:
        xyz = row[1:4]
        if last_xyz is not None and np.linalg.norm(xyz - last_xyz) < spacing:
            continue
        selected.append(row)
        last_xyz = xyz
    return np.asarray(selected, dtype=float)


def write_ascii_pcd(path, points):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w") as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z\n")
        f.write("SIZE 4 4 4\n")
        f.write("TYPE F F F\n")
        f.write("COUNT 1 1 1\n")
        f.write(f"WIDTH {len(points)}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {len(points)}\n")
        f.write("DATA ascii\n")
        for x, y, z in points:
            f.write(f"{x:.5f} {y:.5f} {z:.5f}\n")


def livox_points_to_array(msg, min_range, max_range, min_z, max_z, max_points):
    pts = []
    for pt in msg.points:
        x = float(pt.x)
        y = float(pt.y)
        z = float(pt.z)
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            continue
        if x == 0.0 and y == 0.0 and z == 0.0:
            continue
        r = math.sqrt(x * x + y * y + z * z)
        if r < min_range or r > max_range or z < min_z or z > max_z:
            continue
        pts.append((x, y, z))
    if not pts:
        return np.empty((0, 3), dtype=np.float32)
    arr = np.asarray(pts, dtype=np.float32)
    if max_points > 0 and len(arr) > max_points:
        idx = np.linspace(0, len(arr) - 1, max_points).round().astype(np.int64)
        arr = arr[idx]
    return arr


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--pose-path", required=True)
    ap.add_argument("--lidar-topic", default="/livox/lidar")
    ap.add_argument("--output-dir", required=True)
    ap.add_argument("--spacing", type=float, default=1.0)
    ap.add_argument("--max-time-diff", type=float, default=0.12)
    ap.add_argument("--min-range", type=float, default=0.5)
    ap.add_argument("--max-range", type=float, default=60.0)
    ap.add_argument("--min-z", type=float, default=-3.0)
    ap.add_argument("--max-z", type=float, default=3.0)
    ap.add_argument("--max-points", type=int, default=2500)
    args = ap.parse_args()

    out = Path(args.output_dir)
    scan_dir = out / "scans"
    out.mkdir(parents=True, exist_ok=True)
    scan_dir.mkdir(parents=True, exist_ok=True)

    keyposes = select_keyposes(load_poses(args.pose_path), args.spacing)
    pose_out = out / "keyframe_poses.txt"
    manifest_rows = []
    target_idx = 0
    last_msg = None
    saved = 0

    with rosbag.Bag(args.bag) as bag:
        for _, msg, _ in bag.read_messages(topics=[args.lidar_topic]):
            if target_idx >= len(keyposes):
                break
            msg_t = msg.header.stamp.to_sec()
            while target_idx < len(keyposes) and msg_t >= keyposes[target_idx, 0]:
                candidates = []
                if last_msg is not None:
                    candidates.append(last_msg)
                candidates.append((msg_t, msg))
                best_t, best_msg = min(candidates, key=lambda item: abs(item[0] - keyposes[target_idx, 0]))
                dt = abs(best_t - keyposes[target_idx, 0])
                if dt <= args.max_time_diff:
                    pts = livox_points_to_array(
                        best_msg, args.min_range, args.max_range, args.min_z, args.max_z, args.max_points
                    )
                    if len(pts) > 50:
                        pcd_name = scan_dir / f"key_{saved:06d}.pcd"
                        write_ascii_pcd(pcd_name, pts)
                        row = keyposes[target_idx].copy()
                        row[0] = best_t
                        manifest_rows.append(row)
                        saved += 1
                target_idx += 1
            last_msg = (msg_t, msg)

    with pose_out.open("w") as f:
        for row in manifest_rows:
            f.write(" ".join(f"{v:.9f}" for v in row) + "\n")

    print(f"saved_keyframes={saved}")
    print(f"pose_path={pose_out}")
    print(f"scan_dir={scan_dir}")


if __name__ == "__main__":
    main()
