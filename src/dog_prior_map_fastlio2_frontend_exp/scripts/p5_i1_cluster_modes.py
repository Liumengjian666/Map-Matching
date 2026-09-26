#!/usr/bin/env python3
"""Deterministic, GT-blind clustering of P5-I1 fixed-input NDT searches."""

import argparse
import csv
import math
import time
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np


def read_csv(path):
    with Path(path).open(newline="") as stream:
        return list(csv.DictReader(stream))


def write_csv(path, rows):
    if not rows:
        raise RuntimeError(f"refusing to write empty CSV: {path}")
    with Path(path).open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def pose(value):
    fields = list(map(float, value.split(";")))
    if len(fields) == 16:
        return np.asarray(fields, dtype=float).reshape((4, 4))
    x, y, z, qx, qy, qz, qw = fields
    q = np.array([qx, qy, qz, qw], dtype=float)
    q /= np.linalg.norm(q)
    xq, yq, zq, wq = q
    r = np.array([
        [1 - 2 * (yq*yq + zq*zq), 2 * (xq*yq - zq*wq), 2 * (xq*zq + yq*wq)],
        [2 * (xq*yq + zq*wq), 1 - 2 * (xq*xq + zq*zq), 2 * (yq*zq - xq*wq)],
        [2 * (xq*zq - yq*wq), 2 * (yq*zq + xq*wq), 1 - 2 * (xq*xq + yq*yq)],
    ])
    t = np.eye(4)
    t[:3, :3] = r
    t[:3, 3] = [x, y, z]
    return t


def rot_angle(a, b):
    rel = a[:3, :3].T @ b[:3, :3]
    c = np.clip((np.trace(rel) - 1.0) * 0.5, -1.0, 1.0)
    return math.degrees(math.acos(c))


def separation(a, b):
    return float(np.linalg.norm(a[:3, 3] - b[:3, 3])), rot_angle(a, b)


def se3_log(transform):
    r = transform[:3, :3]
    c = np.clip((np.trace(r) - 1.0) * 0.5, -1.0, 1.0)
    theta = math.acos(c)
    if theta < 1e-8:
        w = 0.5 * np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])
    elif abs(math.sin(theta)) < 1e-8:
        eigvals, eigvecs = np.linalg.eig(r)
        axis = np.real(eigvecs[:, np.argmin(np.abs(eigvals - 1.0))])
        axis /= np.linalg.norm(axis)
        w = axis * theta
    else:
        w = theta / (2.0 * math.sin(theta)) * np.array(
            [r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])
    wx, wy, wz = w
    W = np.array([[0, -wz, wy], [wz, 0, -wx], [-wy, wx, 0]], dtype=float)
    if theta < 1e-8:
        V = np.eye(3) + 0.5 * W + W @ W / 6.0
    else:
        V = np.eye(3) + (1 - math.cos(theta)) / theta**2 * W + (theta - math.sin(theta)) / theta**3 * (W @ W)
    rho = np.linalg.solve(V, transform[:3, 3])
    return np.concatenate([rho, w])


def connected_clusters(runs, trans_tau, rot_tau):
    """Deterministic complete-link clustering under both SE(3) cutoffs."""
    poses = [pose(row["final_pose_matrix16"]) for row in runs]
    count = len(runs)
    if count == 0:
        return []

    # Single-link connected components can chain A~B~C into one alleged mode
    # while A and C violate the threshold. Complete-link merges only when every
    # cross-cluster pair remains within both cutoffs. Seed index provides a
    # deterministic tie-break and objective scores never affect membership.
    capacity = 2 * count - 1
    distances = np.full((capacity, capacity), np.inf, dtype=float)
    members = {i: [i] for i in range(count)}
    first_seed = {i: int(runs[i]["seed_index"]) for i in range(count)}
    for i in range(count):
        for j in range(i + 1, count):
            dt, dr = separation(poses[i], poses[j])
            distances[i, j] = distances[j, i] = max(dt / trans_tau, dr / rot_tau)

    active = list(range(count))
    next_cluster = count
    while len(active) > 1:
        active.sort(key=lambda cluster_id: first_seed[cluster_id])
        active_distances = distances[np.ix_(active, active)]
        upper_mask = np.triu(np.ones(active_distances.shape, dtype=bool), k=1)
        closest = float(np.min(active_distances[upper_mask]))
        if closest > 1.0:
            break
        tied_pairs = np.argwhere(upper_mask & (active_distances == closest))
        left_pos, right_pos = tied_pairs[0]
        left, right = active[int(left_pos)], active[int(right_pos)]
        merged_members = sorted(members[left] + members[right],
                                key=lambda i: (int(runs[i]["seed_index"]), i))
        first_seed[next_cluster] = min(first_seed[left], first_seed[right])
        members[next_cluster] = merged_members
        for other in active:
            if other in (left, right):
                continue
            merged_distance = max(distances[left, other], distances[right, other])
            distances[next_cluster, other] = distances[other, next_cluster] = merged_distance
        active = [cluster_id for cluster_id in active if cluster_id not in (left, right)]
        active.append(next_cluster)
        next_cluster += 1

    clusters = []
    for cluster_id in sorted(active, key=lambda cid: first_seed[cid]):
        indices = members[cluster_id]
        clusters.append([(runs[i], poses[i]) for i in indices])
    return clusters


def summarize_cluster(cluster, frame, threshold_name, denominator):
    runs = [entry[0] for entry in cluster]
    poses = [entry[1] for entry in cluster]
    best = max(runs, key=lambda row: float(row["raw_ndt_score_sum"]))
    representative = pose(best["final_pose_matrix16"])
    scores = np.array([float(row["raw_ndt_score_sum"]) for row in runs])
    fitness = np.array([float(row["fitness"]) for row in runs])
    t_spread, r_spread = [], []
    for other in poses:
        dt, dr = separation(representative, other)
        t_spread.append(dt)
        r_spread.append(dr)
    max_pair_t = max((separation(poses[i], poses[j])[0]
                      for i in range(len(poses)) for j in range(i + 1, len(poses))), default=0.0)
    max_pair_r = max((separation(poses[i], poses[j])[1]
                      for i in range(len(poses)) for j in range(i + 1, len(poses))), default=0.0)
    stable = len(runs) >= 5 and len(runs) / max(denominator, 1) >= 0.02
    baseline_pose = pose(frame["saved_raw_pose_xyz_q_xyzw"])
    bdt, bdr = separation(representative, baseline_pose)
    # The primary lattice contains exactly one unperturbed seed (T_seed=T0).
    # Its converged result is the official baseline replay for this fixed
    # cloud/target/config. Associate the baseline with the one cluster that
    # actually contains this seed; proximity alone can match several bounded
    # clusters near a threshold boundary.
    baseline_contains = any(
        row["seed_domain"] == "PLANAR" and
        abs(float(row["seed_dx_m"])) < 1e-12 and
        abs(float(row["seed_dy_m"])) < 1e-12 and
        abs(float(row["seed_yaw_deg"])) < 1e-12
        for row in runs
    )
    composition = Counter(row["seed_domain"] for row in runs)
    return {
        "frame_id": frame["frame_id"], "transaction_id": frame["transaction_id"],
        "time_s": frame["time_s"], "segment": frame["segment"],
        "selection_labels": frame["selection_labels"], "threshold_set": threshold_name,
        "cluster_id": "", "seed_count": len(runs),
        "converged_seed_denominator": denominator,
        "basin_fraction": len(runs) / max(denominator, 1),
        "stable_mode_candidate": int(stable),
        "representative_pose_xyz_q_xyzw": best["final_pose_xyz_q_xyzw"],
        "representative_pose_matrix16": best["final_pose_matrix16"],
        "representative_score": float(best["raw_ndt_score_sum"]),
        "best_score": float(np.max(scores)), "median_score": float(np.median(scores)),
        "median_fitness": float(np.median(fitness)),
        "translation_spread_max_from_representative_m": max(t_spread, default=0.0),
        "rotation_spread_max_from_representative_deg": max(r_spread, default=0.0),
        "cluster_translation_diameter_m": max_pair_t,
        "cluster_rotation_diameter_deg": max_pair_r,
        "seed_domain_composition": ";".join(f"{k}:{composition[k]}" for k in sorted(composition)),
        "baseline_raw_translation_to_representative_m": bdt,
        "baseline_raw_rotation_to_representative_deg": bdr,
        "baseline_raw_in_cluster": int(baseline_contains),
        "objective_rank": 0,
    }


def cluster_frame(frame, rows, tau_p):
    converged = [row for row in rows if int(row["converged"]) == 1]
    output = []
    specs = [("strict", tau_p * 0.5, 1.0), ("primary", tau_p, 2.0),
             ("loose", tau_p * 2.0, 4.0)]
    for name, t_tau, r_tau in specs:
        clusters = connected_clusters(converged, t_tau, r_tau) if converged else []
        summaries = [summarize_cluster(c, frame, name, len(converged)) for c in clusters]
        summaries.sort(key=lambda item: (-item["best_score"], item["cluster_id"]))
        for rank, item in enumerate(summaries, 1):
            item["cluster_id"] = f"{name[0].upper()}{rank:02d}"
            item["objective_rank"] = rank
            item["stable_mode_candidate"] = int(
                item["seed_count"] >= 5 and item["basin_fraction"] >= 0.02
            )
        output.extend(summaries)
    return output


def write_pre_gt_outputs(out_dir, manifest_path, seed_path):
    clustering_start = time.perf_counter()
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    frames = read_csv(manifest_path)
    seeds = read_csv(seed_path)
    seed_by_frame = defaultdict(list)
    for row in seeds:
        seed_by_frame[row["frame_id"]].append(row)
    if set(seed_by_frame) != {frame["frame_id"] for frame in frames}:
        raise RuntimeError("seed run frame set differs from frozen manifest")
    all_clusters, frame_summaries, pairs, scatter_rows, curvature_requests = [], [], [], [], []
    frame_pose_map = {}
    for frame in frames:
        frame_runs = seed_by_frame[frame["frame_id"]]
        if any(row.get("input_bag_sha256") != frame.get("input_bag_sha256") or
               row.get("input_map_sha256") != frame.get("input_map_sha256") for row in frame_runs):
            raise RuntimeError(f"seed-run input provenance mismatch: {frame['frame_id']}")
        if any(row["source_hash_expected"] != row["source_hash_actual"] for row in frame_runs):
            raise RuntimeError(f"source hash mismatch in seed runs: {frame['frame_id']}")
        tau_p = max(0.15, 0.25 * 0.8)
        clusters = cluster_frame(frame, frame_runs, tau_p)
        all_clusters.extend(clusters)
        primary = [c for c in clusters if c["threshold_set"] == "primary"]
        stable = [c for c in primary if c["stable_mode_candidate"]]
        frame_summaries.append({
            "frame_id": frame["frame_id"], "transaction_id": frame["transaction_id"],
            "time_s": frame["time_s"], "segment": frame["segment"],
            "selection_labels": frame["selection_labels"], "seed_run_count": len(frame_runs),
            "converged_seed_count": sum(int(r["converged"]) == 1 for r in frame_runs),
            "nonconverged_seed_count": sum(int(r["converged"]) == 0 for r in frame_runs),
            "K_primary_all_clusters": len(primary), "K_primary_stable_modes": len(stable),
            "K1": int(len(stable) == 1), "K2": int(len(stable) == 2),
            "K_ge_3": int(len(stable) >= 3), "stable_basin_entropy": "",
            "normalized_stable_basin_entropy": "", "max_inter_mode_translation_m": "",
            "max_inter_mode_rotation_deg": "", "baseline_cluster_id": "",
            "baseline_cluster_stable": "", "baseline_cluster_seed_count": "",
            "primary_tau_translation_m": tau_p, "primary_tau_rotation_deg": 2.0,
        })
        baseline_clusters = [c for c in primary if c["baseline_raw_in_cluster"]]
        curvature_modes = list(stable)
        for mode in baseline_clusters:
            if mode not in curvature_modes:
                curvature_modes.append(mode)
        if curvature_modes:
            for mode in curvature_modes:
                curvature_requests.append({
                    "frame_id": frame["frame_id"], "transaction_id": frame["transaction_id"],
                    "time_s": frame["time_s"], "segment": frame["segment"],
                    "selection_labels": frame["selection_labels"],
                    "cluster_id": mode["cluster_id"], "seed_count": mode["seed_count"],
                    "basin_fraction": mode["basin_fraction"],
                    "representative_score": mode["representative_score"],
                    "stable_mode_candidate": mode["stable_mode_candidate"],
                    "curvature_reason": ("STABLE_MODE" if mode["stable_mode_candidate"] else "BASELINE_CLUSTER_NONSTABLE"),
                    "source_hash_expected": frame["ndt_source_cloud_hash"],
                    "input_bag_sha256": frame["input_bag_sha256"],
                    "input_map_sha256": frame["input_map_sha256"],
                    "representative_pose_xyz_q_xyzw": mode["representative_pose_xyz_q_xyzw"],
                    "representative_pose_matrix16": mode["representative_pose_matrix16"],
                })
        frame_pose_map[frame["frame_id"]] = {c["cluster_id"]: c for c in stable}
        baseline_modes = baseline_clusters
        if baseline_modes:
            frame_summaries[-1]["baseline_cluster_id"] = baseline_modes[0]["cluster_id"]
            frame_summaries[-1]["baseline_cluster_stable"] = baseline_modes[0]["stable_mode_candidate"]
            frame_summaries[-1]["baseline_cluster_seed_count"] = baseline_modes[0]["seed_count"]

        denom = sum(c["seed_count"] for c in stable)
        if stable and denom:
            probabilities = np.array([c["seed_count"] / denom for c in stable], dtype=float)
            entropy = -float(np.sum(probabilities * np.log(probabilities))) if len(stable) > 1 else 0.0
            frame_summaries[-1]["stable_basin_entropy"] = entropy
            frame_summaries[-1]["normalized_stable_basin_entropy"] = (
                entropy / math.log(len(stable)) if len(stable) > 1 else ""
            )
            ref = pose(max(stable, key=lambda c: c["best_score"])["representative_pose_matrix16"])
            tangent_vectors = []
            for mode, probability in zip(stable, probabilities):
                transform = np.linalg.inv(ref) @ pose(mode["representative_pose_matrix16"])
                xi = se3_log(transform)
                tangent_vectors.append(xi)
            tangent_vectors = np.stack(tangent_vectors)
            mean = np.sum(probabilities[:, None] * tangent_vectors, axis=0)
            centered = tangent_vectors - mean
            scatter = sum(probability * np.outer(xi, xi) for probability, xi in zip(probabilities, centered))
            eigvals, eigvecs = np.linalg.eigh(scatter)
            for i in range(6):
                scatter_rows.append({
                    "frame_id": frame["frame_id"], "time_s": frame["time_s"],
                    "ref_mode_id": max(stable, key=lambda c: c["best_score"])["cluster_id"],
                    "basin_weighted_inter_mode_scatter_trace": float(np.trace(scatter)),
                    "translation_block_trace": float(np.trace(scatter[:3, :3])),
                    "rotation_block_trace": float(np.trace(scatter[3:, 3:])),
                    "xi_translation_mean_m": ";".join(map(str, mean[:3])),
                    "xi_rotation_mean_rad": ";".join(map(str, mean[3:])),
                    "scatter_eigenvalue_rank_desc": i + 1,
                    "scatter_eigenvalue": float(eigvals[::-1][i]),
                    "scatter_eigenvector": ";".join(map(str, eigvecs[:, ::-1][:, i])),
                })
            for i, a in enumerate(stable):
                for b in stable[i + 1:]:
                    ta, tb = pose(a["representative_pose_matrix16"]), pose(b["representative_pose_matrix16"])
                    dt, dr = separation(ta, tb)
                    xi = se3_log(np.linalg.inv(ta) @ tb)
                    pairs.append({
                        "frame_id": frame["frame_id"], "time_s": frame["time_s"],
                        "mode_i": a["cluster_id"], "mode_j": b["cluster_id"],
                        "translation_separation_m": dt, "rotation_separation_deg": dr,
                        "log_se3_translation": ";".join(map(str, xi[:3])),
                        "log_se3_rotation_rad": ";".join(map(str, xi[3:])),
                        "score_i": a["best_score"], "score_j": b["best_score"],
                        "basin_fraction_i": a["basin_fraction"],
                        "basin_fraction_j": b["basin_fraction"],
                    })
            frame_summaries[-1]["max_inter_mode_translation_m"] = max(
                (p["translation_separation_m"] for p in pairs if p["frame_id"] == frame["frame_id"]), default=0.0)
            frame_summaries[-1]["max_inter_mode_rotation_deg"] = max(
                (p["rotation_separation_deg"] for p in pairs if p["frame_id"] == frame["frame_id"]), default=0.0)

    clustering_seconds = time.perf_counter() - clustering_start
    write_csv(out_dir / "mode_clusters.csv", all_clusters)
    write_csv(out_dir / "frame_mode_summary_pre_gt.csv", frame_summaries)
    if pairs:
        write_csv(out_dir / "mode_pairwise_separation.csv", pairs)
    else:
        write_csv(out_dir / "mode_pairwise_separation.csv", [{
            "frame_id": "", "time_s": "", "mode_i": "", "mode_j": "",
            "translation_separation_m": "", "rotation_separation_deg": "",
            "log_se3_translation": "", "log_se3_rotation_rad": "", "score_i": "",
            "score_j": "", "basin_fraction_i": "", "basin_fraction_j": "",
        }])
    if scatter_rows:
        write_csv(out_dir / "basin_weighted_inter_mode_scatter.csv", scatter_rows)
    if curvature_requests:
        write_csv(out_dir / "curvature_requests.csv", curvature_requests)
    write_pre_gt_plot(out_dir, frame_summaries)
    with (out_dir / "PRE_GT_FREEZE.md").open("w") as stream:
        stream.write("# P5-I1 pre-GT mode discovery freeze\n\n")
        stream.write("GT was not opened or read in this stage. Seed generation, objective ranking, clustering, stable-mode tagging, basin entropy, inter-mode separation and scatter were completed using fixed scan inputs only.\n\n")
        stream.write("Runtime topic bag: `/media/jian/HIKVISION/paper rosbag/SuperLoc/Floor01/results/p3_r10b_fix1_floor01_full_rerun_20260926/floor01_fix1_runtime_topics.bag`.\n")
        stream.write("Frozen H1 map: `/tmp/floor01_candidates/floor01_h1_map.pcd`; PCD points 549637; final fixed target points 549606 after finite filtering and two 0.15 m voxel passes.\n")
        stream.write("NDT: PCL `NormalDistributionsTransform<PointXYZ,PointXYZ>`, package version `1.10.0+dfsg-5ubuntu1`; resolution 0.8 m, step 0.08, transformation epsilon 0.001, max iterations 40; target-cell radius search. Full fixed target; no local crop.\n\n")
        stream.write(f"Frames: {len(frames)}; total runs: {len(seeds)}; stable curvature requests: {len(curvature_requests)}.\n")
        if not frames or not seeds:
            raise RuntimeError("missing frozen frames or seed runs for provenance freeze")
        stream.write(f"Runtime-topic bag SHA256: `{frames[0]['input_bag_sha256']}`.\n")
        stream.write(f"Prior-map PCD SHA256: `{frames[0]['input_map_sha256']}`.\n")
        stream.write("Primary thresholds: translation <= 0.20 m and rotation <= 2 deg; strict = half translation/1 deg; loose = 0.40 m/4 deg. Stable-mode diagnostic tag: >=5 converged seeds and >=2% of converged seeds in the fixed seed domain.\n")
        stream.write("Clustering: deterministic complete-link agglomeration using max(translation/tau_translation, rotation/tau_rotation); clusters merge only if all cross-cluster pairs satisfy both cutoffs. Equal-distance ties use the minimum original seed index. This prevents single-link chaining.\n")
        stream.write("Seed schedule: each frame has 245 planar + 18 axial seeds; targeted frames add 48 wide-ring seeds with yaw {-15, 0, +15} deg. Pose perturbation is right/body: T_seed = T0 Exp(delta).\n")
        stream.write(f"GT-blind clustering/scatter script wall time: {clustering_seconds:.6f} s (includes deterministic clustering and table aggregation; excludes plotting/write-out).\n")
    write_csv(out_dir / "clustering_runtime.csv", [{
        "stage": "GT_BLIND_CLUSTERING_AND_SCATTER",
        "wall_time_s": clustering_seconds,
        "frames": len(frames), "seed_runs": len(seeds),
        "primary_strict_loose_cluster_rows": len(all_clusters),
    }])
    print(f"FRAMES={len(frames)}")
    print(f"SEED_RUNS={len(seeds)}")
    print(f"PRIMARY_STABLE_MODE_CURVATURE_REQUESTS={len(curvature_requests)}")
    print(f"MULTIMODE_FRAMES={sum(s['K_primary_stable_modes'] > 1 for s in frame_summaries)}")
    print(f"OUTPUT={out_dir}")


def write_pre_gt_plot(out_dir, summaries):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    xs = np.array([float(row["time_s"]) for row in summaries])
    modes = np.array([int(row["K_primary_stable_modes"]) for row in summaries])
    entropy = np.array([float(row["stable_basin_entropy"])
                        if row["stable_basin_entropy"] else np.nan for row in summaries])
    fig, ax = plt.subplots(figsize=(11, 4))
    ax.step(xs, modes, where="mid", color="#2166ac")
    ax.set(xlabel="Elapsed time (s)", ylabel="Stable mode count K", title="P5-I1: GT-blind stable-mode count")
    ax.grid(True, alpha=0.25); fig.tight_layout(); fig.savefig(out_dir / "01_mode_count_over_time.png", dpi=160); plt.close(fig)
    fig, ax = plt.subplots(figsize=(11, 4))
    ax.plot(xs, entropy, marker="o", ms=3, color="#b2182b")
    ax.set(xlabel="Elapsed time (s)", ylabel="Basin entropy (nats)", title="GT-blind empirical stable-basin entropy")
    ax.grid(True, alpha=0.25); fig.tight_layout(); fig.savefig(out_dir / "02_basin_entropy_over_time.png", dpi=160); plt.close(fig)
    fig, ax = plt.subplots(figsize=(11, 4))
    ys = np.array([float(row["max_inter_mode_translation_m"])
                   if row["max_inter_mode_translation_m"] else np.nan for row in summaries])
    ax.plot(xs, ys, marker="o", ms=3, color="#4d9221")
    ax.set(xlabel="Elapsed time (s)", ylabel="Max inter-mode translation (m)", title="GT-blind primary-cluster separation")
    ax.grid(True, alpha=0.25); fig.tight_layout(); fig.savefig(out_dir / "03_inter_mode_separation_over_time.png", dpi=160); plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--seed-runs", required=True)
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()
    write_pre_gt_outputs(args.output_dir, args.manifest, args.seed_runs)


if __name__ == "__main__":
    main()
