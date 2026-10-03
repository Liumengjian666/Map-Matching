#!/usr/bin/env python3
"""Freeze current-baseline inputs and contracts before P5-I1 multi-start replay.

This script is deliberately GT-blind: it does not accept or open a GT path.
"""

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path


EXPECTED_IDS = [120, 244, 368, 616, 740, 838, 839, 864, 924, 925, 1111,
                1235, 1359, 1497, 1498, 1556, 1557, 1606, 1730, 1854,
                2102, 2226, 2350, 2598, 2722, 2846, 3094, 3217, 3341,
                3631, 3796, 3962]
TARGETED_IDS = {838, 839, 924, 925, 1497, 1498, 1556, 1557}
EXPECTED_BAG_SHA = "860d41e88038165be469420b2c9e4db1e164ded4d58952b7b507ee01a4b0a9db"
EXPECTED_MAP_SHA = "2b571af236738a0664befacdc9c783246e991416a8915bcfebb9d2dc074e4570"


def sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def canonical_hash(value):
    payload = json.dumps(value, sort_keys=True, separators=(",", ":"),
                         ensure_ascii=True).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def read_csv(path):
    with Path(path).open(newline="") as stream:
        return list(csv.DictReader(stream))


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def run(args):
    manifest_rows = read_csv(args.manifest)
    export_rows = read_csv(args.cohort_export)
    manifest_by_tx = {int(row["transaction_id"]): row for row in manifest_rows}
    export_by_tx = {int(row["transaction_id"]): row for row in export_rows}
    require(len(manifest_rows) == 32 and set(manifest_by_tx) == set(EXPECTED_IDS),
            "P5_I1_manifest_is_not_the_frozen_32_frame_cohort")
    require(len(export_rows) == 32 and set(export_by_tx) == set(EXPECTED_IDS),
            "current_baseline_export_is_not_the_frozen_32_frame_cohort")

    map_sha = sha256_file(args.map)
    bag_sha = sha256_file(args.bag)
    require(map_sha == EXPECTED_MAP_SHA, "map_sha256_does_not_match_frozen_P5_I1_map")
    require(bag_sha == EXPECTED_BAG_SHA, "runtime_topic_bag_sha256_does_not_match_frozen_P5_I1_bag")

    source_impl_sha = sha256_file(args.ndt_source)
    source_header_sha = sha256_file(args.ndt_header)
    runner_sha = sha256_file(args.runner_source)
    seed_source_sha = sha256_file(args.seed_source)
    cluster_source_sha = sha256_file(args.cluster_source)
    source_config = {
        "finite_xyz": True,
        "range_min_m_inclusive": 0.5,
        "range_max_m_inclusive": 80.0,
        "voxel_leaf_m": 0.25,
        "voxel_order": "PCL_VOXEL_GRID",
        "max_points": 1400,
        "cap_rule": "evenly_spaced_indices_round_llround_endpoints_inclusive",
        "implementation_source_sha256": source_impl_sha,
        "header_sha256": source_header_sha,
    }
    target_config = {
        "finite_xyz": True,
        "first_voxel_leaf_m": 0.15,
        "second_voxel_leaf_m": 0.15,
        "voxel_order": "PCL_VOXEL_GRID_FIRST_MAP_VOXEL_THEN_TARGET_VOXEL",
        "target_grid_resolution_m": 0.8,
        "set_resolution_before_set_input_target": True,
        "implementation_source_sha256": source_impl_sha,
        "header_sha256": source_header_sha,
    }
    source_config_sha = canonical_hash(source_config)
    target_config_sha = canonical_hash(target_config)
    objective_contract = {
        "baseline_sha": "70aa6859657c92404751cd354bd4292e9d30c617",
        "map_sha256": map_sha,
        "target_preprocessing_config_sha256": target_config_sha,
        "target_point_count_expected": 549606,
        "source_preprocessing_config_sha256": source_config_sha,
        "configured_resolution_m": 0.8,
        "actual_target_grid_leaf_m": [0.80000001192092896] * 3,
        "step_size": 0.08,
        "transformation_epsilon": 1e-5,
        "maximum_iterations": 80,
        "pcl_version": "1.10.0+dfsg-5ubuntu1",
        "objective": "PCL_1_10_NDT_SCORE_SUM_MAXIMIZED_BY_ALIGN",
        "scan_cloud_frame": "LIDAR_SCAN_END_FRAME",
        "scan_reference_time": "SCAN_END",
        "deskew": "CURRENT_BASELINE_SCAN_END_DESKEW",
        "runtime_ndt_source_sha256": source_impl_sha,
        "runtime_ndt_header_sha256": source_header_sha,
    }
    objective_sha = canonical_hash(objective_contract)

    seed_schedule = {
        "version": "P5_I1_SEED_SCHEDULE_V1_UNCHANGED",
        "seed_source_sha256": seed_source_sha,
        "planar": {
            "translation_offsets_resolution_multipliers": [-2.0, -1.0, -0.5, 0.0, 0.5, 1.0, 2.0],
            "yaw_deg": [-10.0, -5.0, 0.0, 5.0, 10.0],
            "loop_order": "ix_iy_iyaw",
        },
        "axial": {
            "z_resolution_multipliers": [-1.0, -0.5, 0.5, 1.0],
            "roll_deg": [-5.0, -2.0, 2.0, 5.0],
            "pitch_deg": [-5.0, -2.0, 2.0, 5.0],
            "yaw_deg": [-15.0, 15.0],
            "x_y_resolution_multipliers": [-3.0, 3.0],
        },
        "targeted_wide": {
            "radius_m": [max(3.0 * 0.8, 1.5), max(5.0 * 0.8, 3.0)],
            "angles_deg": [0, 45, 90, 135, 180, 225, 270, 315],
            "yaw_deg": [-15.0, 0.0, 15.0],
            "frame_ids": sorted(TARGETED_IDS),
        },
        "perturbation": "RIGHT_BODY_T_SEED_EQUALS_T0_TIMES_EXP_SE3_DELTA",
        "per_frame_seed_counts": {"control": 263, "targeted": 311},
        "total_alignments": 8800,
    }
    seed_schedule_sha = canonical_hash(seed_schedule)
    clustering_contract = {
        "algorithm": "DETERMINISTIC_COMPLETE_LINK_MAX_NORMALIZED_SE3_CUTOFF",
        "implementation_sha256": cluster_source_sha,
        "primary_translation_cutoff_m": 0.20,
        "primary_rotation_cutoff_deg": 2.0,
        "strict_translation_cutoff_m": 0.10,
        "strict_rotation_cutoff_deg": 1.0,
        "loose_translation_cutoff_m": 0.40,
        "loose_rotation_cutoff_deg": 4.0,
        "stable_min_converged_seeds": 5,
        "stable_min_fraction_of_converged": 0.02,
        "tie_break": "minimum_original_seed_index",
    }
    clustering_sha = canonical_hash(clustering_contract)

    output_rows = []
    frame_provenance = []
    for tx in EXPECTED_IDS:
        manifest = manifest_by_tx[tx]
        exported = export_by_tx[tx]
        require(manifest["input_bag_sha256"] == bag_sha and
                manifest["input_map_sha256"] == map_sha,
                f"historical_manifest_input_digest_mismatch_tx_{tx}")
        require(int(exported["stamp_ns"]) == int(manifest["scan_end_ns"]),
                f"scan_end_timestamp_mismatch_tx_{tx}")
        targeted = tx in TARGETED_IDS
        require(int(exported["wide_targeted"]) == int(targeted),
                f"targeted_seed_schedule_mismatch_tx_{tx}")
        require(int(exported["raw_point_count"]) > 0 and
                int(exported["prepared_source_point_count"]) > 0,
                f"empty_source_cloud_tx_{tx}")
        require(int(exported["target_point_count"]) == 549606,
                f"target_point_count_mismatch_tx_{tx}")
        require(abs(float(exported["configured_resolution_m"]) - 0.8) < 1e-12 and
                all(abs(float(exported[f"target_grid_leaf_{axis}_m"]) - 0.8) < 1e-6
                    for axis in ("x", "y", "z")) and
                abs(float(exported["step_size"]) - 0.08) < 1e-12 and
                abs(float(exported["transformation_epsilon"]) - 1e-5) < 1e-12 and
                int(exported["maximum_iterations"]) == 80,
                f"runtime_ndt_configuration_mismatch_tx_{tx}")
        cloud_path = Path(exported["raw_cloud_file"]).resolve()
        expected_file = (Path(args.source_dir) / f"raw_tx_{tx}.xyzf").resolve()
        require(cloud_path == expected_file and cloud_path.is_file(),
                f"raw_cloud_path_or_file_mismatch_tx_{tx}")
        require(cloud_path.stat().st_size == int(exported["raw_point_count"]) * 12,
                f"raw_cloud_binary_size_mismatch_tx_{tx}")
        raw_cloud_sha = sha256_file(cloud_path)
        initial_pose = exported["initial_pose_xyz_q_xyzw"]
        terminal_pose = exported["raw_terminal_pose_xyz_q_xyzw"]
        initial_pose_sha = canonical_hash({"stamp_ns": int(exported["stamp_ns"]),
                                           "pose_xyz_q_xyzw": initial_pose})
        per_frame_objective = canonical_hash({
            "objective_provenance_sha256": objective_sha,
            "raw_source_sha256": raw_cloud_sha,
            "raw_point_count": int(exported["raw_point_count"]),
            "prepared_source_hash_fnv64": int(exported["prepared_source_hash"]),
            "prepared_source_point_count": int(exported["prepared_source_point_count"]),
        })
        search_sha = canonical_hash({
            "objective_provenance_sha256": per_frame_objective,
            "initial_pose_sha256": initial_pose_sha,
            "seed_schedule_sha256": seed_schedule_sha,
            "clustering_config_sha256": clustering_sha,
        })
        row = {
            "transaction_id": tx,
            "stamp_ns": int(exported["stamp_ns"]),
            "frame_id": manifest["frame_id"],
            "selection_labels": manifest["selection_labels"],
            "segment": manifest["segment"],
            "time_s": manifest["time_s"],
            "wide_targeted": int(targeted),
            "raw_cloud_file": str(cloud_path),
            "raw_source_sha256": raw_cloud_sha,
            "input_bag_sha256": bag_sha,
            "input_map_sha256": map_sha,
            "raw_point_count": int(exported["raw_point_count"]),
            "prepared_source_hash": int(exported["prepared_source_hash"]),
            "prepared_source_point_count": int(exported["prepared_source_point_count"]),
            "target_point_count": int(exported["target_point_count"]),
            "configured_resolution_m": float(exported["configured_resolution_m"]),
            "target_grid_leaf_x_m": float(exported["target_grid_leaf_x_m"]),
            "target_grid_leaf_y_m": float(exported["target_grid_leaf_y_m"]),
            "target_grid_leaf_z_m": float(exported["target_grid_leaf_z_m"]),
            "step_size": float(exported["step_size"]),
            "transformation_epsilon": float(exported["transformation_epsilon"]),
            "maximum_iterations": int(exported["maximum_iterations"]),
            "initial_pose_xyz_q_xyzw": initial_pose,
            "raw_terminal_pose_xyz_q_xyzw": terminal_pose,
            "initial_pose_sha256": initial_pose_sha,
            "source_preprocessing_config_sha256": source_config_sha,
            "target_preprocessing_config_sha256": target_config_sha,
            "objective_provenance_sha256": per_frame_objective,
            "search_provenance_sha256": search_sha,
        }
        output_rows.append(row)
        frame_provenance.append({
            "transaction_id": tx,
            "frame_id": manifest["frame_id"],
            "stamp_ns": int(exported["stamp_ns"]),
            "raw_cloud_sha256": raw_cloud_sha,
            "raw_point_count": int(exported["raw_point_count"]),
            "prepared_source_hash_fnv64": int(exported["prepared_source_hash"]),
            "prepared_source_point_count": int(exported["prepared_source_point_count"]),
            "target_point_count": int(exported["target_point_count"]),
            "initial_pose_xyz_q_xyzw": initial_pose,
            "initial_pose_sha256": initial_pose_sha,
            "raw_terminal_pose_xyz_q_xyzw": terminal_pose,
            "objective_provenance_sha256": per_frame_objective,
            "search_provenance_sha256": search_sha,
        })

    require(sum(311 if tx in TARGETED_IDS else 263 for tx in EXPECTED_IDS) == 8800,
            "frozen_seed_manifest_does_not_sum_to_8800")
    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    cohort_path = out_dir / "cohort_frozen.csv"
    with cohort_path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(output_rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(output_rows)

    inputs = {
        "map_pcd": {"path": str(Path(args.map).resolve()), "sha256": map_sha},
        "runtime_topic_bag": {"path": str(Path(args.bag).resolve()), "sha256": bag_sha},
        "raw_timed_scan_index": {"path": str(Path(args.raw_scan_index).resolve()),
                                 "sha256": sha256_file(args.raw_scan_index)},
        "raw_timed_point_bin": {"path": str(Path(args.raw_scan_bin).resolve()),
                                "sha256": sha256_file(args.raw_scan_bin)},
        "imu_csv": {"path": str(Path(args.imu).resolve()), "sha256": sha256_file(args.imu)},
        "filter_scans_csv": {"path": str(Path(args.filter_scans).resolve()),
                             "sha256": sha256_file(args.filter_scans)},
        "params_txt": {"path": str(Path(args.params).resolve()), "sha256": sha256_file(args.params)},
        "p7_runner_source": {"path": str(Path(args.runner_source).resolve()), "sha256": runner_sha},
        "current_frame_ndt_source": {"path": str(Path(args.ndt_source).resolve()),
                                     "sha256": source_impl_sha},
        "current_frame_ndt_header": {"path": str(Path(args.ndt_header).resolve()),
                                     "sha256": source_header_sha},
        "p5_i1_seed_source": {"path": str(Path(args.seed_source).resolve()),
                              "sha256": seed_source_sha},
        "p5_i1_clustering_source": {"path": str(Path(args.cluster_source).resolve()),
                                    "sha256": cluster_source_sha},
    }
    provenance = {
        "contract": "DUAL_U_R1_CURRENT_BASELINE_SAME_OBJECTIVE_MULTI_START",
        "gt_accessed": False,
        "baseline_sha": objective_contract["baseline_sha"],
        "objective_provenance_sha256": objective_sha,
        "search_provenance_sha256": canonical_hash({
            "objective_provenance_sha256": objective_sha,
            "seed_schedule_sha256": seed_schedule_sha,
            "clustering_config_sha256": clustering_sha,
        }),
        "objective_contract": objective_contract,
        "source_preprocessing_contract": source_config,
        "target_preprocessing_contract": target_config,
        "seed_schedule": seed_schedule,
        "seed_schedule_sha256": seed_schedule_sha,
        "clustering_contract": clustering_contract,
        "clustering_config_sha256": clustering_sha,
        "runner_source_sha256": runner_sha,
        "inputs": inputs,
        "cohort_frame_count": len(output_rows),
        "planned_alignments": 8800,
        "frames": frame_provenance,
        "historical_p5_i1_note": (
            "Historical P5-I1 terminal evidence is not reused; only its predeclared "
            "32-frame manifest, deterministic seed schedule, and clustering contract are reused."
        ),
    }
    provenance_path = out_dir / "objective_provenance.json"
    provenance_path.write_text(json.dumps(provenance, indent=2, sort_keys=True) + "\n")
    print(f"FRAMES={len(output_rows)} PLANNED_ALIGNMENTS=8800")
    print(f"MAP_SHA256={map_sha}")
    print(f"BAG_SHA256={bag_sha}")
    print(f"OBJECTIVE_PROVENANCE_SHA256={objective_sha}")
    print(f"SEARCH_PROVENANCE_SHA256={provenance['search_provenance_sha256']}")
    print(f"COHORT={cohort_path}")
    print(f"PROVENANCE={provenance_path}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--cohort-export", required=True)
    parser.add_argument("--source-dir", required=True)
    parser.add_argument("--map", required=True)
    parser.add_argument("--bag", required=True)
    parser.add_argument("--raw-scan-index", required=True)
    parser.add_argument("--raw-scan-bin", required=True)
    parser.add_argument("--imu", required=True)
    parser.add_argument("--filter-scans", required=True)
    parser.add_argument("--params", required=True)
    parser.add_argument("--ndt-source", required=True)
    parser.add_argument("--ndt-header", required=True)
    parser.add_argument("--runner-source", required=True)
    parser.add_argument("--seed-source", required=True)
    parser.add_argument("--cluster-source", required=True)
    parser.add_argument("--output-dir", required=True)
    run(parser.parse_args())


if __name__ == "__main__":
    main()
