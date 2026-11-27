"""Measure empirical repeat agreement of five fixed, physically acquired Omni scans."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import cKDTree

VOXEL_M = 0.10
CROP_M = 12.0
RANGES = (("all", 0.5, 12), ("0.5–3", 0.5, 3), ("3–6", 3, 6), ("6–10", 6, 10), ("10–12", 10, 12))
K = 16
MAX_DISTANCE_M = 0.15
MAX_PATCH_RADIUS_M = 0.35
MAX_NORMAL_FRACTION = 0.02
MIN_SECOND_EIGEN_M2 = 0.0001


def voxel_centroids(xyz):
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    rad = np.linalg.norm(xyz, axis=1)
    xyz = xyz[(rad >= 0.5) & (rad <= CROP_M)].astype(np.float64)
    _, index = np.unique(np.floor(xyz / VOXEL_M).astype(np.int32), axis=0, return_inverse=True)
    counts = np.bincount(index)
    return np.column_stack([np.bincount(index, weights=xyz[:, i]) / counts for i in range(3)])


def target_planes(target, tree):
    distances, ids = tree.query(target, k=K)
    patches = target[ids]
    centers = patches.mean(axis=1)
    deviations = patches - centers[:, None, :]
    values, vectors = np.linalg.eigh(np.einsum("...ij,...ik->...jk", deviations, deviations) / K)
    observed = ((distances[:, -1] <= MAX_PATCH_RADIUS_M)
                & (values[:, 1] >= MIN_SECOND_EIGEN_M2)
                & (values[:, 0] / values.sum(axis=1) <= MAX_NORMAL_FRACTION))
    normals = vectors[:, :, 0]
    reverse = np.einsum("ij,ij->i", -centers, normals) < 0
    normals[reverse] *= -1
    return centers, normals, observed


def summarize(name, xyz, tree, centers, normals, observed):
    nearest, indices = tree.query(xyz)
    rad = np.linalg.norm(xyz, axis=1)
    support = (nearest <= MAX_DISTANCE_M) & observed[indices]
    signed = np.einsum("ij,ij->i", xyz - centers[indices], normals[indices])
    output = []
    for label, lo, hi in RANGES:
        keep = (rad >= lo) & (rad < hi)
        supported = keep & support
        residual = signed[supported]
        record = {
            "scope": name, "range_bin_m": label, "query_voxels": int(keep.sum()),
            "nearest_le_15cm": int((keep & (nearest <= MAX_DISTANCE_M)).sum()),
            "planar_supported": int(supported.sum()),
            "planar_supported_ratio": float(supported.sum() / max(1, keep.sum())),
            "unconditional_nn_median_cm": float(np.median(nearest[keep])*100) if keep.any() else "",
            "unconditional_nn_p90_cm": float(np.quantile(nearest[keep], 0.9)*100) if keep.any() else "",
            "plane_signed_median_cm": float(np.median(residual)*100) if residual.size else "",
            "plane_mad_cm": float(np.median(np.abs(residual - np.median(residual)))*100) if residual.size else "",
            "plane_abs_p90_cm": float(np.quantile(np.abs(residual), 0.9)*100) if residual.size else "",
        }
        output.append(record)
    return output, nearest, signed, support


def evaluate(input_dir, output):
    manifest = json.loads((input_dir / "OMNI_INPUT_MANIFEST.json").read_text())
    path = input_dir / "omni_observations.npz"
    assert hashlib.sha256(path.read_bytes()).hexdigest() == manifest["outputs"][path.name]
    inputs = np.load(path)
    reference = voxel_centroids(inputs["aligned_O164"][:, :3])
    tree = cKDTree(reference)
    centers, normals, observed = target_planes(reference, tree)
    all_stats = []
    clouds = []
    residuals = []
    relative = inputs["T_nominal_from_scan"]
    stamps = inputs["stamp_ns"]
    temporal_rows = []
    for j, raw_id in enumerate(inputs["raw_ordinals"]):
        label = f"O{raw_id}"
        xyz = voxel_centroids(inputs[f"aligned_{label}"][:, :3])
        record = manifest["acquisitions"][label]
        temporal_rows.append({
            "raw_ordinal": int(raw_id), "stamp_ns": int(stamps[j]),
            "relative_time_s": float((stamps[j] - stamps[-1]) / 1e9),
            "active_role": "reference_acquisition" if raw_id == 164 else "causal_prior_acquisition",
            "aligned_voxels": len(xyz), "original_points": record["points"],
            "translation_m": record["shift_m"], "rotation_deg": record["relative_rotation_deg"],
            **{f"T_reference_from_scan_{r}{c}": float(relative[j, r, c]) for r in range(4) for c in range(4)},
        })
        if raw_id == 164:
            continue
        stats, nearest, signed, support = summarize(label, xyz, tree, centers, normals, observed)
        all_stats.extend(stats)
        clouds.append(xyz)
        residuals.append((label, nearest, signed, support))
    stats, *_ = summarize("pooled_diagnostic_only", np.concatenate(clouds), tree, centers, normals, observed)
    all_stats.extend(stats)
    for name, rows in [("omni_repeatability.csv", all_stats), ("omni_temporal_replay.csv", temporal_rows)]:
        with (output / name).open("x", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0]), lineterminator="\n")
            writer.writeheader()
            writer.writerows(rows)
    fig, axes = plt.subplots(1, 3, figsize=(11.2, 3.7), constrained_layout=True)
    ax = axes[0]
    ax.scatter(*reference[:, :2].T, s=0.6, c="#16283f", alpha=0.40, label="O164")
    ax.scatter(*clouds[0][:, :2].T, s=0.6, c="#ce6138", alpha=0.40, label="O160")
    ax.set_aspect("equal", adjustable="box")
    ax.set(xlabel="LiDAR x (m)", ylabel="LiDAR y (m)", title="Causal alignment to O164")
    ax.legend(loc="upper right", markerscale=6, fontsize=8)
    for label, nearest, signed, support in residuals:
        nearest = np.sort(nearest * 100)
        axes[1].plot(nearest, (np.arange(len(nearest)) + 1) / len(nearest), label=label)
        offset = np.sort(np.abs(signed[support]) * 100)
        axes[2].plot(offset, (np.arange(len(offset)) + 1) / len(offset), label=label)
    axes[1].set(xlim=(0, 30), xlabel="All points NN distance (cm)", ylabel="Empirical cumulative fraction",
                title="All source voxels")
    axes[2].set(xlim=(0, 15), xlabel="Absolute point-to-plane offset (cm)", ylabel="Empirical cumulative fraction",
                title="Supported planar neighborhoods")
    for ax in axes[1:]:
        ax.grid(alpha=0.18)
        ax.legend(frameon=False, fontsize=8, loc="lower right")
    fig.savefig(output / "omni_empirical_repeatability.png", dpi=175)
    plt.close(fig)
    for row in all_stats:
        if row["range_bin_m"] == "all" or row["scope"] == "pooled_diagnostic_only":
            print(row)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    evaluate(args.input, args.output)


if __name__ == "__main__":
    main()
