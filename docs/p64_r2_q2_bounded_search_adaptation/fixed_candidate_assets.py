"""Export frozen Q1 candidates and offline reward diagnostics to fresh directories."""
import argparse
import csv
import gzip
import hashlib
import io
import json
import shutil
from collections import defaultdict
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

Q1 = Path("/home/jian/p64_r2_q1_ws")
Q1_RUN = Q1 / "run"
FIELDS = [f"T{i}{j}" for i in range(4) for j in range(4)]
COVARIANCE = [f"C{i}{j}" for i in range(3) for j in range(3)]
CANDIDATES = (
    ("H307_N1406", "H307", 1406, "fit", "native_terminal_wrong"),
    ("H307_N1739", "H307", 1739, "fit", "native_terminal_wrong"),
    ("H307_P2648", "H307", 2648, "fit", "park_seed_native_terminal_correct"),
    ("H515_N2425", "H515", 2425, "grid", "native_rank_1_wrong"),
    ("H515_N2200", "H515", 2200, "grid", "native_rank_2_boundary_correct"),
)
PAIRS = (
    ("H307_native_competitors", "H307_N1406", "H307_N1739", "two_distinct_native_terminals_both_incorrect"),
    ("H307_correct_wrong", "H307_P2648", "H307_N1406", "correct_vs_wrong_native_terminals_park_seeded_correct"),
    ("H515_boundary_rank", "H515_N2200", "H515_N2425", "correct_boundary_grid_vs_wrong_top_grid"),
)
CROP_M = 8.0
BIN_M = 0.5
NEUTRAL = 0.05


def sha(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def csvrows(path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def writecsv(path, rows):
    columns = list(dict.fromkeys(k for row in rows for k in row))
    with path.open("x", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def online_paths(q1):
    records = [line.split() for line in (q1 / "input/online.tsv").read_text().splitlines()]
    return {r[0]: (Path(r[1]), Path(r[2])) for r in records}


def plan(q1, path):
    seal = json.loads((q1 / "online_seal.json").read_text())
    for filename in ("input/online.tsv", "official/grid.csv", "official/fit.csv"):
        assert sha(q1 / filename) == seal["files"][filename], filename
    files = online_paths(q1)
    sources = {
        table: {(row["sample_id"], int(row["index"])): row for row in csvrows(q1 / "official" / (table + ".csv"))}
        for table in ("grid", "fit")
    }
    with path.open("x") as stream:
        for candidate, frame, index, table, _ in CANDIDATES:
            source = sources[table][frame, index]
            entries = [candidate, frame, *map(str, files[frame]), *[source[key] for key in FIELDS]]
            assert all(not any(c.isspace() for c in value) for value in entries)
            stream.write("\t".join(entries) + "\n")


def pcd_read(path):
    head, body = path.read_bytes().split(b"DATA binary\n", 1)
    attrs = {row.split(maxsplit=1)[0]: row.split(maxsplit=1)[1]
             for row in head.decode().splitlines()
             if row and not row.startswith("#")}
    assert attrs["FIELDS"] == "x y z intensity"
    assert attrs["SIZE"] == "4 4 4 4" and attrs["TYPE"] == "F F F F"
    return np.frombuffer(body, dtype="<f4").reshape(int(attrs["POINTS"]), 4)[:, :3].astype(float)


def pcd_write(path, xyz, colors=None):
    if colors is None:
        fields, types, num = "x y z", "F F F", 3
        data = np.asarray(xyz, dtype="<f4")
    else:
        fields, types, num = "x y z rgb", "F F F F", 4
        channels = np.asarray(colors, dtype=np.uint32)
        packed = (channels[:, 0] << 16 | channels[:, 1] << 8 | channels[:, 2]).astype("<u4").view("<f4")
        data = np.column_stack((np.asarray(xyz, dtype="<f4"), packed))
    header = (
        f"# .PCD v0.7\nVERSION 0.7\nFIELDS {fields}\nSIZE {' '.join(['4'] * num)}\n"
        f"TYPE {types}\nCOUNT {' '.join(['1'] * num)}\nWIDTH {len(xyz)}\nHEIGHT 1\n"
        f"VIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(xyz)}\nDATA binary\n"
    )
    with path.open("xb") as stream:
        stream.write(header.encode())
        stream.write(data.tobytes())


def transform(points, pose):
    return points @ pose[:3, :3].T + pose[:3, 3]


def local_points(points, pose):
    return transform(points, np.linalg.inv(pose))


def fixed_regions(points, delta):
    grouped = defaultdict(list)
    for key, value in zip(np.floor(points / BIN_M).astype(int), delta):
        grouped[tuple(key)].append(value)
    return [
        dict(ix=i, iy=j, iz=k, points=len(values), sum_delta=float(np.sum(values)),
             sum_abs_delta=float(np.sum(np.abs(values))),
             supports_a=int(np.count_nonzero(np.array(values) > NEUTRAL)),
             supports_b=int(np.count_nonzero(np.array(values) < -NEUTRAL)))
        for (i, j, k), values in sorted(grouped.items())
    ]


def normals(points):
    distances, indices = cKDTree(points).query(points, k=16)
    neighborhoods = points[indices]
    centered = neighborhoods - neighborhoods.mean(axis=1, keepdims=True)
    eigenvalues, eigenvectors = np.linalg.eigh(centered.transpose(0, 2, 1) @ centered)
    keep = (
        (distances[:, -1] <= 0.5)
        & (eigenvalues[:, 1] > 1.e-4)
        & (eigenvalues[:, 0] / np.maximum(eigenvalues[:, 1], 1.e-12) <= 0.2)
    )
    return eigenvectors[keep, :, 0], keep


def percentiles(values):
    return np.quantile(values, [0.5, 0.95]).tolist() if len(values) else [None, None]


def pose_gap(pose):
    return float(np.linalg.norm(pose[:3, 3])), float(np.degrees(Rotation.from_matrix(pose[:3, :3]).magnitude()))


def gzip_copy(source, target):
    with source.open("rb") as stream, target.open("xb") as out:
        with gzip.GzipFile(filename="", fileobj=out, mode="wb", compresslevel=6, mtime=0) as archive:
            shutil.copyfileobj(stream, archive)


def shade(xyz, rgb):
    return np.broadcast_to(rgb, xyz.shape).copy()


def panel_figures(output, candidate_clouds, sources, point_deltas, pairs):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    for pair, a, b, _ in pairs:
        fig, axs = plt.subplots(1, 3, figsize=(13.5, 4.1), layout="constrained")
        for idx, axis in enumerate(axs[:2]):
            key = (a, b)[idx]
            points = candidate_clouds[key]
            source = sources[key.split("_")[0]]
            axis.scatter(points[:, 0], points[:, 2], s=0.45, c="#88919b", alpha=0.33, rasterized=True)
            axis.scatter(source[:, 0], source[:, 2], s=1.45, c="#088a99", alpha=0.62, rasterized=True)
            axis.set_title(key)
            axis.set_aspect("equal")
            axis.set(xlabel="Sensor x (m)", ylabel="Sensor z (m)", xlim=(-4, 4), ylim=(-0.2, 8))
        points, delta, regions = point_deltas[pair]
        clip = max(np.quantile(np.abs(delta), 0.95), 0.01)
        points_plot = axs[2].scatter(points[:, 0], points[:, 2], s=6, c=delta,
                                    cmap="RdBu", vmin=-clip, vmax=clip, rasterized=True)
        axs[2].set(title="Native contribution Δs = A − B", xlabel="Sensor x (m)", ylabel="Sensor z (m)",
                   xlim=(-4, 4), ylim=(-0.2, 8), aspect="equal")
        fig.colorbar(points_plot, ax=axs[2], label="Native reward difference")
        fig.suptitle(pair + " | map and observed scan; scores are not structure differences")
        fig.savefig(output / (pair + "_overlay.png"), dpi=150)
        plt.close(fig)

        largest = sorted(regions, key=lambda row: abs(row["sum_delta"]), reverse=True)[:14]
        fig, axis = plt.subplots(figsize=(9.0, 4.4), layout="constrained")
        labels = [f"({r['ix']},{r['iy']},{r['iz']})" for r in largest][::-1]
        values = [r["sum_delta"] for r in largest][::-1]
        colors = ["#3974a0" if value >= 0 else "#bc5442" for value in values]
        axis.barh(labels, values, color=colors)
        axis.set(title=pair + " | 14 strongest signed native-reward bins",
                 xlabel="Sum of matched reward differences, A − B", ylabel="0.5 m bin (i,j,k)")
        axis.axvline(0, color="#333333", lw=0.7)
        fig.savefig(output / (pair + "_regional_rewards.png"), dpi=150)
        plt.close(fig)


def analyze(q1, export, plan_path, docs, pcd_output):
    pcd_output.mkdir(exist_ok=False)
    asset_output = docs / "r3_fixed_assets"
    asset_output.mkdir(exist_ok=False)
    expected = {row["sample_id"]: row for row in csvrows(Q1 / "docs/p64_r2_q1_candidate_search/qualification_sample_manifest.csv")}
    plan_entries = {row.split()[0]: row.split()[1:] for row in plan_path.read_text().splitlines()}
    summaries = {row["candidate_id"]: row for row in csvrows(export / "score_summary.csv")}
    poses = {key: np.array(row[3:], dtype=float).reshape(4, 4) for key, row in plan_entries.items()}
    q1_fit = {(r["sample_id"], int(r["index"])): r for r in csvrows(q1 / "evaluation/refinement_offline_errors.csv")}
    q1_grid = {(r["sample_id"], int(r["index"])): r for r in csvrows(q1 / "official/grid.csv")}
    q1_labels = {(r["sample_id"], int(r["index"])): r for r in csvrows(q1 / "evaluation/candidate_mode_offline_errors.csv")
                 if r["metric"] == "native" and r["index"]}
    receipt_file = Q1 / "docs/p64_r2_q1_candidate_search/raw_artifact_sha256.txt"
    receipt_line = next(line for line in receipt_file.read_text().splitlines()
                        if line.endswith("  run/reference.tsv"))
    assert sha(q1 / "reference.tsv") == receipt_line.split()[0]
    references = {row.split()[0]: np.array(row.split()[1:], dtype=float).reshape(4, 4)
                  for row in (q1 / "reference.tsv").read_text().splitlines()}
    inputs = online_paths(q1)
    source_xyz = {}
    raw = {}
    for frame in ("H307", "H515"):
        map_path, scan_path = inputs[frame]
        assert sha(map_path) == expected[frame]["map_sha256"]
        assert sha(scan_path) == expected[frame]["prepared_pcd_sha256"]
        assert sha(export / "filtered_map_xyz.bin") == expected[frame]["expected_target_xyz_sha256"]
        path = export / f"{frame}_filtered_source_xyz.bin"
        assert sha(path) == expected[frame]["expected_source_xyz_sha256"]
        source_xyz[frame] = np.fromfile(path, dtype="<f4").reshape(-1, 3).astype(float)
        raw[frame] = pcd_read(scan_path)
        pcd_write(pcd_output / (frame + "_source_native_0p1m.pcd"), source_xyz[frame])
    world = pcd_read(inputs["H307"][0])
    world_cells = csvrows(export / "native_cells_world.csv")
    cells_xyz = np.array([[float(row[key]) for key in ("mx", "my", "mz")] for row in world_cells])
    covariances = np.array([[float(row[key]) for key in COVARIANCE] for row in world_cells]).reshape(-1, 3, 3)

    candidate_rows, scales = [], []
    candidate_clouds = {}
    for candidate, frame, index, table, use in CANDIDATES:
        pose = poses[candidate]
        historical = (q1_fit if table == "fit" else q1_grid)[frame, index]
        summary = summaries[candidate]
        assert float(historical["native"]) == float(summary["native_mean"])
        assert int(historical["associated"]) == int(summary["associated"])
        label = historical if table == "fit" else q1_labels[frame, index]
        translation_error = label["fit_translation_error_m" if table == "fit" else "translation_error_m"]
        rotation_error = label["fit_rotation_error_deg" if table == "fit" else "rotation_error_deg"]
        candidate_rows.append(dict(
            candidate_id=candidate, frame=frame, q1_index=index, source_table=table, qualification=use,
            reference_use="Q1_offline_label_only", translation_error_m=translation_error,
            rotation_error_deg=rotation_error,
            q1_fit_iterations=historical.get("iterations", ""), q1_fit_converged=historical.get("reported_converged", ""),
            geometric_boundary=table == "grid" and bool(
                min(int(historical["ix"]), int(historical["iy"]), int(historical["ia"])) == 0
                or int(historical["ix"]) == 20 or int(historical["iy"]) == 20 or int(historical["ia"]) == 10),
            local_crop_m=CROP_M, map_sha256=expected[frame]["map_sha256"],
            source_pcd_sha256=expected[frame]["prepared_pcd_sha256"],
            source_native_xyz_sha256=expected[frame]["expected_source_xyz_sha256"],
            target_native_xyz_sha256=expected[frame]["expected_target_xyz_sha256"],
            **{key: value for key, value in summary.items() if key != "candidate_id"},
            **dict(zip(FIELDS, pose.flat)),
        ))
        local = local_points(world, pose)
        keep = np.linalg.norm(local, axis=1) <= CROP_M
        candidate_clouds[candidate] = local[keep]
        round_trip = transform(local[keep], pose)
        assert np.max(np.abs(round_trip - world[keep])) < 1.e-10
        pcd_write(pcd_output / (candidate + "_local_map_raw.pcd"), local[keep])
        # Gaussian grid keeps its original full-map voxel aggregation, then transforms
        # cell centers/covariances. Never revoxelize separate candidate maps.
        local_centers = local_points(cells_xyz, pose)
        rotation = np.linalg.inv(pose[:3, :3])
        local_cov = rotation @ covariances @ rotation.T
        local_cells = [
            dict(cell_id=world_cells[j]["cell_id"], points=world_cells[j]["points"],
                 x_L=local_centers[j, 0], y_L=local_centers[j, 1], z_L=local_centers[j, 2],
                 **dict(zip(COVARIANCE, local_cov[j].flat)))
            for j in np.flatnonzero(np.linalg.norm(local_centers, axis=1) <= CROP_M)
        ]
        writecsv(asset_output / (candidate + "_local_gaussians.csv"), local_cells)
        source = source_xyz[frame]
        normal, usable = normals(source)
        reference_in_local = np.linalg.solve(pose, references[frame])
        displacement = transform(source, reference_in_local) - source
        projected = np.abs(np.sum(normal * displacement[usable], axis=1))
        projected_median, projected_p95 = percentiles(projected)
        spatial_median, spatial_p95 = percentiles(np.linalg.norm(displacement, axis=1))
        scales.append(dict(candidate_id=candidate, frame=frame, source_points=len(source),
                           planar_normal_support=int(usable.sum()),
                           abs_normal_offline_median_m=projected_median, abs_normal_offline_p95_m=projected_p95,
                           xyz_offline_median_m=spatial_median, xyz_offline_p95_m=spatial_p95,
                           method="source_normal_16_neighbors_le_0p5m_q1_reference_offline_only"))
    writecsv(docs / "fixed_candidate_manifest.csv", candidate_rows)
    writecsv(docs / "microstructure_offline_projection.csv", scales)
    refinement_receipt = []
    for frame, index in (("H307", 1406), ("H307", 1739), ("H307", 2648), ("H515", 2425), ("H515", 2200)):
        seed = q1_grid[frame, index]
        fit = q1_fit.get((frame, index), {})
        row = dict(frame=frame, q1_index=index,
                   state="Q1_refined" if fit else "Q1_boundary_seed_excluded",
                   initial_native_mean=seed["native"], initial_associated=seed["associated"],
                   initial_denominator=seed["total"],
                   iterations=fit.get("iterations", ""), reported_converged=fit.get("reported_converged", ""),
                   terminal_native_mean=fit.get("native", ""), terminal_associated=fit.get("associated", ""),
                   terminal_denominator=fit.get("total", ""))
        row.update({"initial_" + key: seed[key] for key in FIELDS})
        row.update({"terminal_" + key: fit.get(key, "") for key in FIELDS})
        refinement_receipt.append(row)
    writecsv(docs / "fixed_candidate_q1_refinements.csv", refinement_receipt)

    diagnostic_rows = []
    pair_rows = []
    point_deltas = {}
    for pair, a, b, qualification in PAIRS:
        frame = a.split("_")[0]
        points_a = csvrows(export / (a + "_points.csv"))
        points_b = csvrows(export / (b + "_points.csv"))
        assert len(points_a) == len(points_b) == len(source_xyz[frame])
        assert all(l["point_id"] == r["point_id"] and all(l[k] == r[k] for k in ("x_L", "y_L", "z_L"))
                   for l, r in zip(points_a, points_b))
        points = source_xyz[frame]
        first = np.array([float(record["native_reward"]) for record in points_a])
        second = np.array([float(record["native_reward"]) for record in points_b])
        delta = first - second
        difference = float(summaries[a]["native_sum"]) - float(summaries[b]["native_sum"])
        assert abs(delta.sum() - difference) < 1.e-9
        regions = fixed_regions(points, delta)
        abs_summary = np.sum(np.abs(delta))
        top = sorted((abs(row["sum_delta"]) for row in regions), reverse=True)
        group_signed_abs_total = sum(top)
        weak = np.abs(delta) <= NEUTRAL
        diagnostic_rows.append(dict(
            pair_id=pair, candidate_a=a, candidate_b=b, row_type="summary", points=len(points),
            sum_delta=delta.sum(), sum_abs_delta=abs_summary, mean_delta=delta.mean(),
            supports_a=int(np.count_nonzero(delta > NEUTRAL)), supports_b=int(np.count_nonzero(delta < -NEUTRAL)),
            near_neutral_0p05=int(weak.sum()), near_neutral_0p01=int(np.count_nonzero(np.abs(delta) <= 0.01)),
            near_neutral_0p10=int(np.count_nonzero(np.abs(delta) <= 0.10)),
            weak_sum_delta=delta[weak].sum(), strong_sum_delta=delta[~weak].sum(),
            weak_sum_delta_0p01=delta[np.abs(delta) <= 0.01].sum(),
            weak_sum_delta_0p10=delta[np.abs(delta) <= 0.10].sum(),
            max_point_share_abs=float(np.max(np.abs(delta)) / abs_summary),
            top1pct_point_share_abs=float(np.sort(np.abs(delta))[-int(np.ceil(len(delta) / 100)):].sum() / abs_summary),
            max_region_share_abs_signed=top[0] / group_signed_abs_total,
            top5_region_share_abs_signed=sum(top[:5]) / group_signed_abs_total,
            abs_reward_cancellation=1 - abs(delta.sum()) / abs_summary,
        ))
        diagnostic_rows.extend(dict(pair_id=pair, candidate_a=a, candidate_b=b, row_type="region", **r) for r in regions)
        with (asset_output / (pair + "_point_deltas.csv.gz")).open("xb") as out:
            with gzip.GzipFile(filename="", fileobj=out, mode="wb", mtime=0) as archive:
                with io.TextIOWrapper(archive, encoding="utf-8", newline="") as file:
                    writer = csv.writer(file, lineterminator="\n")
                    writer.writerow(("point_id", "x_L", "y_L", "z_L", "s_a", "s_b", "delta_s_a_minus_b"))
                    writer.writerows((i, *xyz, sa, sb, d)
                                     for i, (xyz, sa, sb, d) in enumerate(zip(points, first, second, delta)))
        relative = np.linalg.solve(poses[a], poses[b])
        translation, rotation = pose_gap(relative)
        pair_rows.append(dict(
            pair_id=pair, candidate_a=a, candidate_b=b, qualification=qualification,
            translation_m=translation, rotation_deg=rotation, delta_native_mean=delta.mean(),
            guarantee="Q1_sealed_pose_identity_recomputed_no_new_fit",
            **dict(zip([f"T_A_B{i}{j}" for i in range(4) for j in range(4)], relative.flat)),
        ))
        first_map, second_map = candidate_clouds[a], candidate_clouds[b]
        rgb_first = shade(first_map, (40, 112, 180))
        rgb_second = shade(second_map, (212, 141, 41))
        rgb_scan = shade(points, (32, 174, 139))
        pcd_write(pcd_output / (pair + "_maps_and_scan.pcd"),
                  np.vstack((first_map, second_map, points)), np.vstack((rgb_first, rgb_second, rgb_scan)))
        color_limit = max(np.quantile(np.abs(delta), 0.95), 0.01)
        signed = np.clip(delta / color_limit, -1, 1)
        intensity = (255 * (1 - np.abs(signed))).astype(np.uint8)
        red = np.where(signed < 0, 255, intensity)
        blue = np.where(signed > 0, 255, intensity)
        pcd_write(pcd_output / (pair + "_native_reward_delta.pcd"), points,
                  np.column_stack((red, intensity, blue)))
        point_deltas[pair] = (points, delta, regions)
    writecsv(docs / "discriminative_score_diagnostics.csv", diagnostic_rows)
    writecsv(docs / "fixed_candidate_pairs.csv", pair_rows)
    panel_figures(asset_output, candidate_clouds, source_xyz, point_deltas, PAIRS)

    for candidate, frame, _, _, _ in CANDIDATES:
        pose = poses[candidate]
        if frame == "H515":
            cloud = transform(raw[frame], pose)
            local_world = transform(candidate_clouds[candidate], pose)
            pcd_write(pcd_output / (candidate + "_match_world.pcd"), np.vstack((local_world, cloud)),
                      np.vstack((shade(local_world, (158, 158, 158)), shade(cloud, (36, 174, 172)))))
    for path in sorted(pcd_output.glob("*.pcd")):
        gzip_copy(path, asset_output / (path.name + ".gz"))
    for path in sorted(export.glob("*_contributions.csv")):
        gzip_copy(path, asset_output / (path.name + ".gz"))
    for path in sorted(export.glob("*_points.csv")):
        gzip_copy(path, asset_output / (path.name + ".gz"))
    gzip_copy(export / "native_cells_world.csv", asset_output / "native_cells_world.csv.gz")
    inventory = []
    for path in sorted(asset_output.iterdir()):
        inventory.append(dict(asset=str(path.relative_to(docs)), kind="tracked_candidate_asset",
                              bytes=path.stat().st_size, sha256=sha(path)))
    for path in sorted(pcd_output.glob("*.pcd")):
        inventory.append(dict(asset=str(path), kind="viewable_uncompressed_pcd",
                              bytes=path.stat().st_size, sha256=sha(path)))
    for path in sorted({file for pair in (inputs["H307"], inputs["H515"]) for file in pair}):
        inventory.append(dict(asset=str(path), kind="immutable_Q1_input",
                              bytes=path.stat().st_size, sha256=sha(path)))
    inventory.append(dict(asset=str(plan_path), kind="Q1_sealed_candidate_only_plan",
                          bytes=plan_path.stat().st_size, sha256=sha(plan_path)))
    writecsv(docs / "r3_asset_inventory.csv", inventory)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    make_plan = sub.add_parser("plan")
    make_plan.add_argument("path", type=Path)
    analysis = sub.add_parser("analyze")
    for name in ("export", "plan_path", "docs", "pcd_output"):
        analysis.add_argument(name, type=Path)
    args = parser.parse_args()
    if args.command == "plan":
        plan(Q1_RUN, args.path)
    else:
        analyze(Q1_RUN, args.export, args.plan_path, args.docs, args.pcd_output)


if __name__ == "__main__":
    main()
