"""Render H515 fixed-grid and uniform C3 terminal only after Q2 seals the fit."""
import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from fixed_candidate_assets import (
    CROP_M, FIELDS, Q1_RUN, csvrows, gzip_copy, normals, online_paths,
    pcd_read, pcd_write, percentiles, pose_gap, sha, shade, transform, writecsv,
)


def matrix(record):
    return np.array([record[k] for k in FIELDS], dtype=float).reshape(4, 4)


def run(seal_path, fit_path, docs, pcd_dir):
    key = str(fit_path.resolve().relative_to(seal_path.resolve().parent))
    receipt = json.loads(seal_path.read_text())
    assert sha(fit_path) == receipt["files"][key]
    archive = docs / "r3_fixed_assets"
    inventory_path = docs / "r3_asset_inventory.csv"
    assert archive.is_dir() and inventory_path.is_file()
    known = {row["candidate_id"]: row for row in csvrows(docs / "fixed_candidate_manifest.csv")}
    fits = [row for row in csvrows(fit_path)
            if row["sample_id"] == "H515" and row["method"] == "S0" and row["index"] == "2200"]
    assert len(fits) == 1 and fits[0]["policy"] == "C3"
    fit = fits[0]
    expected_reference = next(
        line.split()[0] for line in (Q1_RUN.parent / "docs/p64_r2_q1_candidate_search/raw_artifact_sha256.txt").read_text().splitlines()
        if line.endswith("  run/reference.tsv")
    )
    assert sha(Q1_RUN / "reference.tsv") == expected_reference
    q1_reference = next(line.split()[1:] for line in (Q1_RUN / "reference.tsv").read_text().splitlines()
                        if line.startswith("H515\t"))
    reference = np.array(q1_reference, dtype=float).reshape(4, 4)
    map_path, scan_path = online_paths(Q1_RUN)["H515"]
    world = pcd_read(map_path)
    raw_scan = pcd_read(scan_path)
    centroid_xyz = np.fromfile(Q1_RUN / "gate/H515_source_xyz.bin", dtype="<f4").reshape(-1, 3).astype(float)
    normal, usable = normals(centroid_xyz)
    seed = known["H515_N2200"]
    series = (
        ("Q1 Native rank 1", known["H515_N2425"]),
        ("Q1 retained yaw-boundary seed", seed),
        ("Q2 C3 native fit from boundary", fit),
    )
    output = []
    fig, axs = plt.subplots(1, 3, figsize=(14, 4.4), layout="constrained")
    lower = reference[:2, 3] - 4.5
    upper = reference[:2, 3] + 4.5
    for axis, (name, row) in zip(axs, series):
        pose = matrix(row)
        translation, angle = pose_gap(np.linalg.solve(pose, reference))
        displacement = transform(centroid_xyz, np.linalg.solve(pose, reference)) - centroid_xyz
        p50, p95 = percentiles(np.abs(np.sum(normal * displacement[usable], axis=1)))
        center = pose[:3, 3]
        local = world[np.linalg.norm(world - center, axis=1) <= CROP_M]
        predicted = transform(centroid_xyz, pose)
        axis.scatter(local[:, 0], local[:, 1], s=0.7, c="#777c83", alpha=0.35, rasterized=True)
        axis.scatter(predicted[:, 0], predicted[:, 1], s=1.7, c="#147c88", alpha=0.49, rasterized=True)
        axis.scatter(*reference[:2, 3], marker="x", c="#bc482f", s=32)
        axis.scatter(*center[:2], marker="+", c="#15427f", s=44)
        axis.set(title=f"{name}\n{translation:.3f} m / {angle:.2f}° vs reference",
                 xlabel="Map x (m)", ylabel="Map y (m)", aspect="equal")
        axis.set_xlim(lower[0], upper[0])
        axis.set_ylim(lower[1], upper[1])
        output.append(dict(
            pose_role=name, native_mean=row.get("native_mean") or row["native"], associated=row["associated"],
            source_points=row.get("source_points") or row["total"], reported_converged=row.get("reported_converged", ""),
            iterations=row.get("iterations", ""), mode_preserved=row.get("mode_preserved", ""),
            drift_translation_m=row.get("drift_translation_m", ""),
            drift_rotation_deg=row.get("drift_rotation_deg", ""),
            error_translation_m=translation, error_rotation_deg=angle,
            abs_normal_reference_p50_m=p50, abs_normal_reference_p95_m=p95,
            correct_at_0p15m_2deg=bool(translation <= 0.15 and angle <= 2.0),
            **dict(zip(FIELDS, pose.flat)),
        ))
    fig.suptitle("H515 boundary control | map gray, observed scan teal, reference ×, pose +")
    figure = archive / "H515_boundary_before_after.png"
    assert not figure.exists()
    fig.savefig(figure, dpi=150)
    plt.close(fig)
    pose = matrix(fit)
    local = world[np.linalg.norm(world - pose[:3, 3], axis=1) <= CROP_M]
    scan = transform(raw_scan, pose)
    pcd = pcd_dir / "H515_Q2_C3_terminal_match_world.pcd"
    pcd_write(pcd, np.vstack((local, scan)),
              np.vstack((shade(local, (153, 153, 153)), shade(scan, (36, 174, 172)))))
    archived = archive / (pcd.name + ".gz")
    gzip_copy(pcd, archived)
    final_csv = docs / "H515_boundary_fit_receipt.csv"
    writecsv(final_csv, output)
    for path, kind in (
        (figure, "tracked_Q2_boundary_diagnostic"),
        (pcd, "viewable_uncompressed_Q2_boundary_pcd"),
        (archived, "tracked_Q2_boundary_diagnostic"),
        (final_csv, "tracked_Q2_sealed_boundary_receipt"),
    ):
        assert all(str(path) != row["asset"] for row in csvrows(inventory_path))
        with inventory_path.open("a") as stream:
            csv.writer(stream, lineterminator="\n").writerow([
                path.relative_to(docs) if docs in path.parents else path, kind, path.stat().st_size, sha(path),
            ])


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for field in ("seal", "fits", "docs", "pcd_dir"):
        parser.add_argument(field, type=Path)
    args = parser.parse_args()
    run(args.seal, args.fits, args.docs.resolve(), args.pcd_dir.resolve())
