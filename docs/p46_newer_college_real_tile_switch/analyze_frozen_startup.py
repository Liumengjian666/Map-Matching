#!/usr/bin/env python3
"""Compare the first 30 frozen P44/P45 scans and locate early EKF jumps."""

import csv
import json
import math
import sys
from pathlib import Path


P44 = Path("/media/jian/HIKVISION/paper rosbag/NewerCollege/.p44_300frame/headless_300")
P45 = Path("/media/jian/HIKVISION/paper rosbag/NewerCollege/.p45_local_map/formal300")


def rows(directory, name):
    with (directory / name).open(newline="") as handle:
        return list(csv.DictReader(handle))


def translation(row, prefix):
    return tuple(float(row[prefix + axis]) for axis in ("tx", "ty", "tz"))


def position(row):
    return tuple(float(row[axis]) for axis in ("px", "py", "pz"))


def q_angle_deg(a, b):
    qa = tuple(float(a["q" + axis]) for axis in ("x", "y", "z", "w"))
    qb = tuple(float(b["q" + axis]) for axis in ("x", "y", "z", "w"))
    dot = abs(sum(x * y for x, y in zip(qa, qb)))
    return math.degrees(2.0 * math.acos(min(1.0, max(-1.0, dot))))


def ndt_q(row, prefix):
    return {"q" + axis: row[prefix + "q" + axis] for axis in ("x", "y", "z", "w")}


def distance(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def first30_ndt(directory):
    ndt = rows(directory, "ndt_diagnostics.csv")[:30]
    map_path = directory / "local_map_diagnostics.csv"
    maps = rows(directory, "local_map_diagnostics.csv")[:30] if map_path.exists() else [{} for _ in ndt]
    oosm = [r for r in rows(directory, "ekf_prediction_diagnostics.csv")
            if r["event_type"] == "OOSM_REPLAY_COMPLETE"][:30]
    if not (len(ndt) == len(maps) == len(oosm) == 30):
        raise RuntimeError("expected 30 corresponding NDT/map/OOSM rows")
    return ndt, maps, oosm


def high_rate(directory):
    return [r for r in rows(directory, "localization_trajectory.csv")
            if r["source"] == "EKF_HIGH_RATE"]


def max_jump(states, end_ns):
    selected = [r for r in states if int(r["stamp_ns"]) <= end_ns]
    candidates = []
    for before, after in zip(selected, selected[1:]):
        dt_ns = int(after["stamp_ns"]) - int(before["stamp_ns"])
        candidates.append((distance(position(before), position(after)), dt_ns,
                           before["stamp_ns"], after["stamp_ns"]))
    if not candidates:
        return None
    return max(candidates)


def main():
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).parent
    p44_ndt, p44_map, p44_oosm = first30_ndt(P44)
    p45_ndt, p45_map, p45_oosm = first30_ndt(P45)
    if [r["frame_index"] for r in p44_ndt] != [r["frame_index"] for r in p45_ndt]:
        raise RuntimeError("P44/P45 frame indices differ")

    fields = [
        "frame_index", "scan_stamp_ns", "cloud_hash_p44", "cloud_hash_p45",
        "initial_guess_delta_m", "initial_guess_delta_deg", "raw_result_delta_m",
        "raw_result_delta_deg", "p44_correction_m", "p45_correction_m",
        "p44_correction_deg", "p45_correction_deg", "p44_fitness", "p45_fitness",
        "p44_iterations", "p45_iterations", "p45_map_update", "p45_update_reason",
        "p45_tile_count", "p45_target_points", "p44_oosm_rewrite_m",
        "p45_oosm_rewrite_m", "p44_oosm_rewrite_deg", "p45_oosm_rewrite_deg",
        "p44_replay_imu_count", "p45_replay_imu_count",
    ]
    comparison = []
    for a, b, ma, mb, oa, ob in zip(p44_ndt, p45_ndt, p44_map, p45_map,
                                    p44_oosm, p45_oosm):
        guess_a, guess_b = translation(a, "initial_guess_"), translation(b, "initial_guess_")
        raw_a = tuple(float(a["raw_ndt_t" + axis]) for axis in ("x", "y", "z"))
        raw_b = tuple(float(b["raw_ndt_t" + axis]) for axis in ("x", "y", "z"))
        q_guess_a = ndt_q(a, "initial_guess_")
        q_guess_b = ndt_q(b, "initial_guess_")
        q_raw_a = ndt_q(a, "raw_ndt_")
        q_raw_b = ndt_q(b, "raw_ndt_")
        stamp_ns = int(round(float(a["lidar_header_stamp"]) * 1e9))
        comparison.append({
            "frame_index": a["frame_index"], "scan_stamp_ns": stamp_ns,
            "cloud_hash_p44": a["cloud_hash"], "cloud_hash_p45": b["cloud_hash"],
            "initial_guess_delta_m": distance(guess_a, guess_b),
            "initial_guess_delta_deg": q_angle_deg(q_guess_a, q_guess_b),
            "raw_result_delta_m": distance(raw_a, raw_b),
            "raw_result_delta_deg": q_angle_deg(q_raw_a, q_raw_b),
            "p44_correction_m": a["raw_delta_from_guess_translation"],
            "p45_correction_m": b["raw_delta_from_guess_translation"],
            "p44_correction_deg": a["raw_delta_from_guess_rotation_deg"],
            "p45_correction_deg": b["raw_delta_from_guess_rotation_deg"],
            "p44_fitness": a["ndt_fitness"], "p45_fitness": b["ndt_fitness"],
            "p44_iterations": a["ndt_iterations"], "p45_iterations": b["ndt_iterations"],
            "p45_map_update": mb["map_updated"], "p45_update_reason": mb["update_reason"],
            "p45_tile_count": mb["loaded_tile_count"], "p45_target_points": mb["target_points"],
            "p44_oosm_rewrite_m": oa["oosm_rewrite_translation_m"],
            "p45_oosm_rewrite_m": ob["oosm_rewrite_translation_m"],
            "p44_oosm_rewrite_deg": oa["oosm_rewrite_rotation_deg"],
            "p45_oosm_rewrite_deg": ob["oosm_rewrite_rotation_deg"],
            "p44_replay_imu_count": oa["replay_imu_count"],
            "p45_replay_imu_count": ob["replay_imu_count"],
        })
    out.mkdir(parents=True, exist_ok=True)
    with (out / "startup_pose_comparison.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        writer.writerows(comparison)

    h44, h45 = high_rate(P44), high_rate(P45)
    keyed44 = {r["stamp_ns"]: r for r in h44}
    keyed45 = {r["stamp_ns"]: r for r in h45}
    common = sorted(set(keyed44) & set(keyed45), key=int)
    if not common:
        raise RuntimeError("no matched high-rate EKF timestamps")
    divergences = []
    for stamp in common:
        delta = distance(position(keyed44[stamp]), position(keyed45[stamp]))
        if delta > 1e-9:
            divergences.append((stamp, delta))
    last_stamp = int(comparison[-1]["scan_stamp_ns"])
    # The 30th scan ends a little over 2.9 s after initialization; use its
    # contemporaneous high-rate state range, not the full 30 s run.
    window_end_ns = last_stamp + 120_000_000
    jump44 = max_jump(h44, window_end_ns)
    jump45 = max_jump(h45, window_end_ns)
    summary = {
        "frames_compared": 30,
        "first_scan_stamp_ns": comparison[0]["scan_stamp_ns"],
        "frame30_stamp_ns": last_stamp,
        "first_frame_initial_guess_delta_m": comparison[0]["initial_guess_delta_m"],
        "first_frame_initial_guess_delta_deg": comparison[0]["initial_guess_delta_deg"],
        "first_frame_filtered_cloud_hash_equal": comparison[0]["cloud_hash_p44"] == comparison[0]["cloud_hash_p45"],
        "first_frame_raw_ndt_result_delta_m": comparison[0]["raw_result_delta_m"],
        "first_frame_raw_ndt_result_delta_deg": comparison[0]["raw_result_delta_deg"],
        "first_frame_p45_target_points": int(comparison[0]["p45_target_points"]),
        "first_frame_p45_loaded_tiles": int(comparison[0]["p45_tile_count"]),
        "first_frame_oosm_rewrite_p44_m": float(comparison[0]["p44_oosm_rewrite_m"]),
        "first_frame_oosm_rewrite_p45_m": float(comparison[0]["p45_oosm_rewrite_m"]),
        "first_common_high_rate_state_stamp_ns": common[0],
        "first_different_high_rate_state": divergences[0] if divergences else None,
        "high_rate_jump_window_end_ns": window_end_ns,
        "maximum_adjacent_high_rate_jump_p44": jump44,
        "maximum_adjacent_high_rate_jump_p45": jump45,
        "p45_first30_oosm_rewrite_values_m": [float(r["p45_oosm_rewrite_m"]) for r in comparison[:5]],
        "p44_first30_oosm_rewrite_values_m": [float(r["p44_oosm_rewrite_m"]) for r in comparison[:5]],
    }
    (out / "startup_pose_comparison.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
