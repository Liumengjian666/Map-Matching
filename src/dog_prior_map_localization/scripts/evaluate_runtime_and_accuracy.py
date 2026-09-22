#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
机器狗端先验地图定位算法评估脚本。

用途
====
读取算法输出的 rosbag、实时性 CSV、CPU采样文件，生成一份 JSON 指标。

评估指标参考 SLAM/定位论文里常见写法：
1. 轨迹一致性：
   - first_last_error_m：第一帧和最后一帧位姿差。
   - first_window_last_window_error_m：开头窗口和结尾窗口平均位姿差。
   - path_length_m：估计轨迹长度。
2. 输出实时性：
   - high_rate_hz：高频传播位姿频率。
   - corrected_hz：地图匹配修正频率。
3. 算力/资源：
   - cpu_avg_percent / cpu_max_percent：CPU占用。
   - rss_avg_mb / rss_max_mb：常驻内存。
   - avg_update_ms / max_update_ms：单次地图匹配耗时。

注意
====
这里没有GNSS/全站仪/动作捕捉真值，因此暂时不能计算严格ATE/RPE。
当前“首尾同点误差”只适用于用户明确开始和结束回到同一地点的数据集。
"""

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np
import rosbag


def read_odom_bag(bag_path):
    data = {"/dog_livo/odom_high_rate": [], "/dog_livo/odom_corrected": []}
    with rosbag.Bag(str(bag_path)) as bag:
        for topic, msg, _ in bag.read_messages(topics=list(data.keys())):
            data[topic].append(
                [
                    msg.header.stamp.to_sec(),
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                    msg.pose.pose.position.z,
                ]
            )
    return {k: np.asarray(v, dtype=float) for k, v in data.items()}


def trajectory_metrics(arr, window_sec):
    if arr.shape[0] < 2:
        return {"count": int(arr.shape[0])}

    rel_t = arr[:, 0] - arr[0, 0]
    xyz = arr[:, 1:4]
    duration = float(arr[-1, 0] - arr[0, 0])
    diffs = np.diff(xyz, axis=0)
    path_length = float(np.linalg.norm(diffs, axis=1).sum())
    first_last = xyz[-1] - xyz[0]

    start_mask = rel_t <= window_sec
    end_mask = rel_t >= max(0.0, duration - window_sec)
    start_mean = xyz[start_mask].mean(axis=0)
    end_mean = xyz[end_mask].mean(axis=0)
    window_diff = end_mean - start_mean

    return {
        "count": int(arr.shape[0]),
        "duration_s": duration,
        "hz": float((arr.shape[0] - 1) / duration) if duration > 0 else 0.0,
        "path_length_m": path_length,
        "first_last_diff_xyz_m": first_last.round(4).tolist(),
        "first_last_error_m": float(np.linalg.norm(first_last)),
        "first_window_last_window_diff_xyz_m": window_diff.round(4).tolist(),
        "first_window_last_window_error_m": float(np.linalg.norm(window_diff)),
        "start_window_count": int(start_mask.sum()),
        "end_window_count": int(end_mask.sum()),
    }


def read_runtime_csv(path):
    path = Path(path)
    if not path.exists():
        return {}
    rows = list(csv.DictReader(path.open()))
    if not rows:
        return {}

    def vals(key):
        return [float(row[key]) for row in rows if row.get(key, "") not in ("", "nan", "NaN")]

    out = {"runtime_rows": len(rows)}
    for key in [
        "imu_hz",
        "correction_hz",
        "ndt_correction_count",
        "oosm_event_count",
        "deferred_received_count",
        "deferred_processed_count",
        "deferred_over_limit_count",
        "deferred_queue_full_count",
        "max_deferred_queue_size",
        "state_history_size",
        "imu_history_size",
    ]:
        arr = vals(key)
        if arr:
            out[key + "_mean"] = float(np.mean(arr))
            out[key + "_max"] = float(np.max(arr))
            out[key + "_min"] = float(np.min(arr))
    out["ndt_correction_count_last"] = int(rows[-1].get("ndt_correction_count", 0))
    return out


def read_cpu_samples(path):
    path = Path(path)
    if not path.exists():
        return {}
    cpu = []
    mem = []
    rss = []
    for line in path.read_text(errors="ignore").splitlines():
        parts = line.split(None, 5)
        if len(parts) >= 5 and "dog_prior_map_ekf_node_cpp" in line:
            try:
                cpu.append(float(parts[2]))
                mem.append(float(parts[3]))
                rss.append(float(parts[4]) / 1024.0)
            except ValueError:
                pass
    if not cpu:
        return {}
    return {
        "cpu_samples": len(cpu),
        "cpu_avg_percent": float(np.mean(cpu)),
        "cpu_max_percent": float(np.max(cpu)),
        "mem_avg_percent": float(np.mean(mem)),
        "mem_max_percent": float(np.max(mem)),
        "rss_avg_mb": float(np.mean(rss)),
        "rss_max_mb": float(np.max(rss)),
    }


def main():
    parser = argparse.ArgumentParser(description="评估C++先验地图定位的精度和算力需求")
    parser.add_argument("--name", required=True)
    parser.add_argument("--odom-bag", required=True)
    parser.add_argument("--runtime-csv", required=True)
    parser.add_argument("--cpu-samples", required=True)
    parser.add_argument("--window-sec", type=float, default=10.0)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    odom = read_odom_bag(args.odom_bag)
    result = {
        "name": args.name,
        "odom_bag": args.odom_bag,
        "window_sec": args.window_sec,
        "high_rate": trajectory_metrics(odom["/dog_livo/odom_high_rate"], args.window_sec),
        "corrected": trajectory_metrics(odom["/dog_livo/odom_corrected"], args.window_sec),
        "runtime": read_runtime_csv(args.runtime_csv),
        "resource": read_cpu_samples(args.cpu_samples),
    }

    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, ensure_ascii=False, indent=2))
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
