"""Evaluate sealed candidate-search output and draw real-data figures."""

import argparse
import csv
import hashlib
import json
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
from scipy.stats import spearmanr

from peaks import METRICS, POSE_FIELDS, read_grid, support


K_VALUES = (1, 3, 5)
DISTANCE_LIMITS = (0.10, 0.15, 0.20, 0.50)
ANGLE_LIMITS = (1.0, 2.0, 3.0, 5.0)
POINT_RANGES = (2.0, 5.0, 10.0, 20.0)
ALPHA = 0.001


@dataclass
class FrameInput:
    map_path: Path
    scan_path: Path
    prediction: np.ndarray
    original: np.ndarray


def digest(path):
    with path.open("rb") as stream:
        hashed = hashlib.sha256()
        for block in iter(lambda: stream.read(1 << 20), b""):
            hashed.update(block)
    return hashed.hexdigest()


def verify_seal(path, required, manifest):
    receipt = json.loads(path.read_text())
    if digest(manifest) != receipt["manifest_sha256"]:
        raise ValueError("frozen sample manifest hash differs from the seal")
    entries = {}
    for filename, checksum in receipt["files"].items():
        name = Path(filename)
        candidate = path.parent / name
        if not candidate.is_file() or digest(candidate) != checksum:
            raise ValueError(f"sealed file hash mismatch or missing: {name}")
        entries[candidate.resolve()] = checksum
    if missing := {p.resolve() for p in required}.difference(entries):
        raise ValueError(f"seal did not cover required inputs: {sorted(map(str, missing))}")


def transform(row):
    return np.array([float(row[field]) for field in POSE_FIELDS]).reshape(4, 4)


def read_inputs(path):
    result = {}
    for line in path.read_text().splitlines():
        if not line.strip():
            continue
        sample, map_file, scan_file, *numbers = line.split()
        if sample in result or len(numbers) != 32:
            raise ValueError(f"invalid or repeated online input for {sample}")
        prediction, original = np.array(numbers, dtype=float).reshape(2, 4, 4)
        result[sample] = FrameInput(Path(map_file), Path(scan_file), prediction, original)
    return result


def read_reference(path):
    result = {}
    for line in path.read_text().splitlines():
        if not line.strip():
            continue
        sample, *numbers = line.split()
        if sample in result or len(numbers) != 16:
            raise ValueError(f"invalid or repeated reference for {sample}")
        matrices = np.array(numbers, dtype=float).reshape(4, 4)
        if not np.isfinite(matrices).all():
            continue
        if not (
            np.allclose(matrices[3, :], [0, 0, 0, 1], atol=1e-8)
            and np.allclose(matrices[:3, :3].T @ matrices[:3, :3], np.eye(3), atol=1e-5)
            and np.linalg.det(matrices[:3, :3]) > 0
        ):
            continue
        result[sample] = matrices
    return result


def load_csv(path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def write_csv(path, rows, fields=None):
    field_names = fields if fields is not None else list(dict.fromkeys(k for row in rows for k in row))
    if not field_names:
        raise ValueError(f"empty CSV without schema: {path}")
    with path.open("x", newline="") as stream:
        writer = csv.DictWriter(stream, field_names)
        writer.writeheader()
        writer.writerows(rows)


def pose_errors(poses, reference):
    translation = np.linalg.norm(poses[..., :3, 3] - reference[:3, 3], axis=-1)
    relative = poses[..., :3, :3] @ reference[:3, :3].T
    # Project finite float rotations by SVD via SciPy before measuring the
    # angle. This avoids trace-saturation near zero in stored float matrices.
    angle = Rotation.from_matrix(relative).magnitude() * 180.0 / np.pi
    return np.asarray(translation), np.asarray(angle)


def bool_cell(value):
    return str(value).lower() in {"1", "true", "yes"}


def safe_relative(numerator, denominator):
    return numerator / denominator if denominator and np.isfinite(denominator) else None


def continuous_offset_error(grid, reference):
    """Distance of a pose from the bounded x/y/yaw manifold of the actual grid."""
    center = min(grid, key=lambda row: abs(float(row["dx"])) + abs(float(row["dy"])) + abs(float(row["dyaw"])))
    center_pose = transform(center)
    edge_x = max(
        (row for row in grid if float(row["dy"]) == 0 and float(row["dyaw"]) == 0),
        key=lambda row: float(row["dx"]),
    )
    edge_y = max(
        (row for row in grid if float(row["dx"]) == 0 and float(row["dyaw"]) == 0),
        key=lambda row: float(row["dy"]),
    )
    origin = center_pose[:3, 3]
    heading = np.arctan2(center_pose[1, 0], center_pose[0, 0])
    cosine, sine = np.cos(heading), np.sin(heading)
    axes = np.array([[cosine, -sine], [sine, cosine], [0.0, 0.0]])
    # Validate the stated convention against realized boundary poses, using
    # an absolute rounding allowance appropriate to float map coordinates.
    for i, (row, key) in enumerate(((edge_x, "dx"), (edge_y, "dy"))):
        expected = origin + axes[:, i] * float(row[key])
        rounding = np.maximum(1e-6, 2 * np.abs(np.spacing(expected.astype(np.float32))))
        if np.any(np.abs(transform(row)[:3, 3] - expected) > rounding):
            raise ValueError("grid axes differ from the stated heading-local offsets")
    local_xy = axes.T @ (reference[:3, 3] - origin)
    projected = origin + axes @ np.clip(local_xy, -2.0, 2.0)
    trans = float(np.linalg.norm(projected - reference[:3, 3]))

    # For a map-Z rotation R_z(theta) R0, the trace is a*cos(theta)
    # +b*sin(theta)+constant. The optimum lies at the stationary point
    # or one of the interval endpoints.
    relative = reference[:3, :3] @ center_pose[:3, :3].T
    yaw = float(np.degrees(np.arctan2(relative[1, 0] - relative[0, 1], relative[0, 0] + relative[1, 1])))
    admissible = np.array([-5.0, 5.0, np.clip(yaw, -5.0, 5.0)])
    rotations = Rotation.from_euler("z", admissible, degrees=True).as_matrix() @ center_pose[:3, :3]
    angles = Rotation.from_matrix(rotations @ reference[:3, :3].T).magnitude()
    angle = float(np.degrees(angles.min()))
    height = float(reference[2, 3] - origin[2])
    return trans, angle, float(local_xy[0]), float(local_xy[1]), yaw, height


def local_grid_spread(rows, grid_index):
    """Finite-neighborhood normalized Park spread. It is a descriptive scale."""
    lookup = {int(row["index"]): row for row in rows}
    center = lookup[grid_index]
    coordinate = np.array([int(center[key]) for key in ("ix", "iy", "ia")])
    local = [
        row for row in rows
        if all(abs(int(row[key]) - x) <= 1 for key, x in zip(("ix", "iy", "ia"), coordinate))
        and support(row, "park") >= 0.5 and np.isfinite(float(row["park"]))
    ]
    if len(local) < 3:
        return None
    positions = np.array([[float(row[key]) for key in ("dx", "dy", "dyaw")] for row in local])
    scores = np.array([float(row["park"]) for row in local])
    weights = np.exp(ALPHA * (scores - scores.max()))
    weights /= weights.sum()
    mean = weights @ positions
    residual = positions - mean
    variance = (weights[:, None] * residual ** 2).sum(axis=0)
    # Treat this as a bounded neighborhood location spread, not a
    # sensor-covariance calibration or a six-dimensional uncertainty model.
    return float(np.sqrt(variance[:2].sum())), float(np.sqrt(variance[2])), len(local)


def classify_error(continuous, within_grid, within_peaks, within_retained, top_correct):
    if continuous is None:
        return "F"
    if top_correct:
        return "OK"
    if continuous[0] > 0.15 or continuous[1] > 2.0:
        return "A"
    if not within_grid:
        return "B"
    if not within_peaks:
        return "C"
    if not within_retained:
        return "D"
    return "E"


def evaluate_sample(sample, grid, modes, fits, frame, reference, meta):
    poses = np.stack([transform(row) for row in grid])
    grid_indices = {row["index"]: idx for idx, row in enumerate(grid)}
    native = np.array([float(row["native"]) for row in grid])
    park = np.array([float(row["park"]) for row in grid])
    metric_support = {metric: np.array([support(row, metric) >= 0.5 for row in grid]) for metric in METRICS}
    scoring_domain = metric_support["native"] & metric_support["park"] & np.isfinite(native) & np.isfinite(park)
    finite_park = np.isfinite(park)
    if not finite_park.all():
        raise ValueError(f"{sample}: Park likelihood is not finite")
    mass = np.exp(ALPHA * (park - park.max()))
    mass /= mass.sum()
    if not np.allclose(mass, [float(row["park_mass"]) for row in grid], rtol=1e-10, atol=1e-15):
        raise ValueError(f"{sample}: archived Park probability mass differs from the Eq. 5 reconstruction")
    offset = np.array([[float(row[key]) for key in ("dx", "dy", "dyaw")] for row in grid])
    weighted_mean = mass @ offset
    centered = offset - weighted_mean
    covariance = (centered.T * mass) @ centered
    order_summary = {
        "sample_id": sample,
        "shared_supported_grid_count": int(scoring_domain.sum()),
        "park_posterior_mass_sum": float(mass.sum()),
        "park_global_std_x_m": float(np.sqrt(covariance[0, 0])),
        "park_global_std_y_m": float(np.sqrt(covariance[1, 1])),
        "park_global_std_yaw_deg": float(np.sqrt(covariance[2, 2])),
    }
    if scoring_domain.sum() > 1 and np.ptp(native[scoring_domain]) and np.ptp(park[scoring_domain]):
        result = spearmanr(native[scoring_domain], park[scoring_domain])
        order_summary["spearman_common_supported"] = float(result[0])

    continuous = None
    t = r = None
    reference_usable = reference is not None
    original_t = original_r = None
    if reference_usable:
        t, r = pose_errors(poses, reference)
        continuous = continuous_offset_error(grid, reference)
        original_t, original_r = (float(value) for value in pose_errors(frame.original, reference))
        nearest = min(range(len(grid)), key=lambda idx: (max(t[idx] / 0.15, r[idx] / 2.0), t[idx], r[idx]))
        correct = (t <= 0.15) & (r <= 2.0)
    else:
        nearest = None
        correct = np.full(len(grid), False)

    joined_modes, recall_rows, scales = [], [], []
    for metric in METRICS:
        supported = metric_support[metric]
        supported_correct = correct & supported
        metric_modes = [row for row in modes if row["metric"] == metric]
        kept = sorted([row for row in metric_modes if bool_cell(row["retained"])], key=lambda row: int(row["rank"]))
        positions = [grid_indices[row["index"]] for row in kept]
        acceptable_peak = any(correct[grid_indices[row["index"]]] for row in metric_modes if row["index"])
        top_idx = positions[0] if positions else None
        best = None if top_idx is None else grid[top_idx]
        order_summary[f"{metric}_best_grid_index"] = "" if best is None else best["index"]
        metric_values = native if metric == "native" else park
        full_best = int(np.nanargmax(metric_values))
        order_summary[f"{metric}_ungated_best_grid_index"] = grid[full_best]["index"]
        order_summary[f"{metric}_ungated_best_support"] = support(grid[full_best], metric)
        if reference_usable:
            order_summary[f"{metric}_ungated_best_correct_015m_2deg"] = bool(correct[full_best])
        if len(positions) > 1:
            highest = metric_values[positions[0]]
            runner = metric_values[positions[1]]
            order_summary[f"{metric}_top2_raw_gap"] = float(highest - runner)
            order_summary[f"{metric}_top2_relative_gap"] = (
                float((highest - runner) / highest) if metric == "native" and highest else
                float(-np.expm1(ALPHA * (runner - highest))) if metric == "park" else None
            )
        for row in metric_modes:
            enriched = dict(row)
            if row["index"]:
                idx = grid_indices[row["index"]]
                enriched["park_posterior_mass"] = float(mass[idx])
                if reference_usable:
                    enriched.update(translation_error_m=float(t[idx]), rotation_error_deg=float(r[idx]),
                                    reference_correct_015m_2deg=bool(correct[idx]))
            joined_modes.append(enriched)
        strongest_wrong = next((idx for idx in positions if not correct[idx]), None)
        strongest_correct = next((idx for idx in positions if correct[idx]), None)
        if reference_usable:
            order_summary[f"{metric}_rank_first_correct_grid_peak"] = next(
                (rank for rank, idx in enumerate(positions, 1) if correct[idx]), None
            )
            if strongest_wrong is not None and strongest_correct is not None:
                order_summary[f"{metric}_correct_minus_strongest_wrong"] = float(
                    metric_values[strongest_correct] - metric_values[strongest_wrong]
                )
            elif strongest_correct is not None:
                order_summary[f"{metric}_correct_minus_strongest_wrong"] = None
            ordered = sorted(np.where(scoring_domain)[0], key=lambda idx: (-metric_values[idx], int(grid[idx]["index"])))
            order_summary[f"{metric}_rank_first_correct_supported_grid_pose"] = next(
                (rank for rank, idx in enumerate(ordered, 1) if correct[idx]), None
            )

        top_correct = top_idx is not None and bool(correct[top_idx])
        category = classify_error(continuous, bool(correct.any()), acceptable_peak,
                                  bool(correct[positions].any()), top_correct)
        reason = category
        if category == "A":
            reason = "OUTSIDE_BOUNDED_MANIFOLD"
        elif category == "B":
            reason = "GRID_TOO_COARSE"
        elif category == "C":
            reason = "NO_SUPPORTED_LOCAL_PEAK" if not supported_correct.any() else "NO_LOCAL_PEAK_AT_CORRECT_GRID"
        elif category == "D":
            reason = "LOCAL_PEAK_SUPPRESSED_OR_RANK_CAP"
        elif category == "E":
            reason = "CORRECT_CANDIDATE_MISRANKED"
        elif category == "F":
            reason = "REFERENCE_UNAVAILABLE"
        for distance in DISTANCE_LIMITS:
            for angle in ANGLE_LIMITS:
                values = {
                    "sample_id": sample, "group": meta.get("group", ""),
                    "protocol": meta.get("protocol", ""), "metric": metric,
                    "translation_tol_m": distance, "rotation_tol_deg": angle,
                    "reference_valid": reference_usable, "mode_class": metric_modes[0]["mode_class"],
                    "retained_modes": len(positions), "best_grid_index": "" if best is None else best["index"],
                    "primary_failure_class": category, "primary_failure_reason": reason,
                }
                if reference_usable:
                    within = (t <= distance) & (r <= angle)
                    values.update(
                        original_ndt_correct=bool(original_t <= distance and original_r <= angle),
                        original_translation_error_m=original_t, original_rotation_error_deg=original_r,
                        continuous_min_translation_m=continuous[0], continuous_min_rotation_deg=continuous[1],
                        reference_dx=continuous[2], reference_dy=continuous[3], reference_yaw_deg=continuous[4],
                        reference_height_offset_m=continuous[5],
                        reference_inside_grid_range=bool(abs(continuous[2]) <= 2 and abs(continuous[3]) <= 2
                                                         and abs(continuous[4]) <= 5),
                        continuous_manifold_within_tolerance=bool(continuous[0] <= distance and continuous[1] <= angle),
                        any_correct_grid_pose=bool(within.any()),
                        any_correct_supported_grid_pose=bool((within & supported).any()),
                        nearest_grid_translation_m=float(t[nearest]), nearest_grid_rotation_deg=float(r[nearest]),
                        nearest_grid_index=grid[nearest]["index"],
                        retained_top1_translation_m=None if top_idx is None else float(t[top_idx]),
                        retained_top1_rotation_deg=None if top_idx is None else float(r[top_idx]),
                    )
                    for k in K_VALUES:
                        values[f"top{k}_correct"] = bool(within[positions[:k]].any())
                    if positions:
                        nearest_kept = min(positions, key=lambda idx: (max(t[idx] / 0.15, r[idx] / 2.0), t[idx], r[idx]))
                        values["nearest_retained_translation_m"] = float(t[nearest_kept])
                        values["nearest_retained_rotation_deg"] = float(r[nearest_kept])
                recall_rows.append(values)

        if best is not None:
            spread = local_grid_spread(grid, int(best["index"]))
            specifications = []
            if reference_usable:
                specifications.append(("grid_best_absolute", float(t[top_idx]), float(r[top_idx]), "offline_reference"))
            if spread:
                specifications.append(("park_local_spread_at_" + metric + "_best", spread[0], spread[1],
                                       f"descriptive_{spread[2]}_grid_cells"))
            for kind, shift, rotation, source in specifications:
                scales.extend(scale_rows(sample, metric, kind, best["index"], shift, rotation, source))

    fit_results = []
    for fit in fits:
        index = grid_indices[fit["index"]]
        terminal = transform(fit)
        seed_to_fit = pose_errors(terminal, poses[index])
        final_support = support(fit)
        converged = bool_cell(fit["reported_converged"])
        fit_ok = converged and final_support >= 0.5 and int(fit["iterations"]) > 0
        record = dict(fit)
        plane = continuous_offset_error(grid, terminal)
        record.update(
            fit_outside_searched_ranges=bool(abs(plane[2]) > 2.0 + 1e-6 or abs(plane[3]) > 2.0 + 1e-6
                                              or abs(plane[4]) > 5.0 + 1e-6),
            fit_distance_bounded_plane_m=plane[0], fit_rotation_bounded_plane_deg=plane[1],
            seed_to_fit_translation_m=float(seed_to_fit[0]), seed_to_fit_rotation_deg=float(seed_to_fit[1]),
            final_support=final_support, qualified_terminal=fit_ok,
        )
        if reference_usable:
            refit_t, refit_r = (float(value) for value in pose_errors(terminal, reference))
            record.update(
                seed_translation_error_m=float(t[index]), seed_rotation_error_deg=float(r[index]),
                fit_translation_error_m=refit_t, fit_rotation_error_deg=refit_r,
                fit_correct_015m_2deg=bool(refit_t <= 0.15 and refit_r <= 2.0),
            )
            scales.extend(scale_rows(sample, fit["seed_kind"], "refined_absolute", fit["index"],
                                     refit_t, refit_r, "offline_reference"))
        scales.extend(scale_rows(sample, fit["seed_kind"], "seed_refine_displacement", fit["index"],
                                 float(seed_to_fit[0]), float(seed_to_fit[1]), "reference_free_displacement"))
        fit_results.append(record)
    verified_pairs = sum(
        float(pose_errors(transform(first), transform(second))[0]) > 0.30
        or float(pose_errors(transform(first), transform(second))[1]) > 3.0
        for n, first in enumerate(fit_results) for second in fit_results[:n]
        if first["qualified_terminal"] and second["qualified_terminal"]
    )
    order_summary["new_align_count"] = len(fits)
    order_summary["qualified_terminal_count"] = sum(row["qualified_terminal"] for row in fit_results)
    order_summary["separated_terminal_pair_count"] = verified_pairs
    return recall_rows, order_summary, joined_modes, fit_results, scales


def scale_rows(sample, metric, kind, index, translation, angle, provenance):
    radians = np.deg2rad(angle)
    return [
        dict(
            sample_id=sample, metric=metric, kind=kind, index=index,
            translation_m=translation, rotation_deg=angle, range_m=distance,
            rotational_linearized_cm=float(distance * radians * 100),
            rotational_exact_cm=float(2 * distance * np.sin(radians / 2) * 100),
            conservative_sum_cm=float((translation + distance * radians) * 100),
            provenance=provenance,
        )
        for distance in POINT_RANGES
    ]


def read_pcd(path):
    header = {}
    with path.open("rb") as stream:
        for line in stream:
            parts = line.decode("ascii").strip().split()
            if not parts or parts[0].startswith("#"):
                continue
            key, *values = parts
            header[key] = values
            if key == "DATA":
                break
        fields = header["FIELDS"]
        scalars = {"F": "f", "U": "u", "I": "i"}
        counts = [int(x) for x in header.get("COUNT", ["1"] * len(fields))]
        descriptors = [
            (name, f"<{scalars[category]}{size}") if count == 1 else
            (name, f"<{scalars[category]}{size}", (count,))
            for name, category, size, count in zip(fields, header["TYPE"], header["SIZE"], counts)
        ]
        dtype = np.dtype(descriptors)
        encoding = header["DATA"][0]
        if encoding == "binary":
            data = np.frombuffer(stream.read(), dtype)
            cloud = np.stack([data[key] for key in ("x", "y", "z")], axis=1)
        elif encoding == "ascii" and all(count == 1 for count in counts):
            cloud = np.loadtxt(stream, usecols=[fields.index(axis) for axis in ("x", "y", "z")])
        else:
            raise ValueError(f"unsupported PCD for visualization: {path} {encoding}")
    return np.asarray(cloud[np.isfinite(cloud).all(axis=1)], dtype=np.float32)


def mapped(points, matrix):
    return (points @ matrix[:3, :3].T + matrix[:3, 3]).astype(np.float32)


def write_colored_pcd(path, cloud, colors):
    points = np.concatenate(cloud)
    rgb = np.concatenate([np.full(len(xyz), color, dtype="<u4") for xyz, color in zip(cloud, colors)])
    data = np.empty(len(points), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<f4")])
    for axis, coord in enumerate("xyz"):
        data[coord] = points[:, axis]
    data["rgb"] = rgb.view("<f4")
    total = len(data)
    header = (
        "VERSION .7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n"
        f"WIDTH {total}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {total}\nDATA binary\n"
    )
    with path.open("xb") as stream:
        stream.write(header.encode("ascii"))
        stream.write(data.tobytes())


def visualize_overlays(plt, path, frame, reference, grid, modes, fits):
    map_cloud = read_pcd(frame.map_path)
    scan_cloud = read_pcd(frame.scan_path)
    original_center = frame.original[:3, 3]
    local_map = map_cloud[np.linalg.norm(map_cloud[:, :2] - original_center[:2], axis=1) <= 8]
    grid_rows = {row["index"]: row for row in grid}
    selected = [
        ("Historical native NDT", frame.original, "original"),
    ]
    for metric in METRICS:
        best = next((row for row in modes if row["metric"] == metric and row["rank"] == "1"), None)
        if best:
            selected.append((f"{metric.title()} retained peak {best['index']}", transform(grid_rows[best["index"]]), metric))
        else:
            winner = max(grid, key=lambda row: float(row[metric]))
            selected.append((f"{metric.title()} ungated grid argmax {winner['index']}",
                             transform(winner), metric))
    for fit in fits[:4]:
        selected.append((f"Fit from {fit['seed_kind']} {fit['index']}", transform(fit), "fit_" + fit["index"]))
    fig, axes = plt.subplots(2, len(selected), figsize=(3.6 * len(selected), 7.2), squeeze=False,
                             sharex="row", sharey="row", constrained_layout=True)
    point_sample = scan_cloud[::max(1, len(scan_cloud) // 35000)]
    for column, (label, matrix, key) in enumerate(selected):
        points = mapped(point_sample, matrix)
        for row, vertical in enumerate((1, 2)):
            axis = axes[row, column]
            axis.scatter(local_map[:, 0], local_map[:, vertical], c="#747d88", s=0.6, alpha=0.35)
            axis.scatter(points[:, 0], points[:, vertical], c="#007c91", s=0.8, alpha=0.32)
            axis.scatter(matrix[0, 3], matrix[vertical, 3], c="#007c91", marker="+", s=64)
            if reference is not None:
                axis.scatter(reference[0, 3], reference[vertical, 3], marker="*", c="#e6550d", s=85, label="Offline reference")
            axis.set(xlabel="map X (m)", ylabel=f"map {'YZ'[row]} (m)")
            axis.set_aspect("equal", adjustable="box")
            if row == 0:
                axis.set_title(label)
        if key in ("original", "native", "park"):
            write_colored_pcd(path / f"raw309_{key}_overlay.pcd", [local_map, mapped(scan_cloud, matrix)],
                              [0x9199A1, 0x009DB4])
    axes[0, 0].set_xlim(original_center[0] - 5, original_center[0] + 5)
    axes[0, 0].set_ylim(original_center[1] - 5, original_center[1] + 5)
    axes[1, 0].set_xlim(original_center[0] - 5, original_center[0] + 5)
    axes[1, 0].set_ylim(original_center[2] - 2.5, original_center[2] + 2.5)
    fig.suptitle("raw309 | gray = prior map, cyan = actual scan, orange star = offline reference")
    fig.savefig(path / "raw309_registered_scan_overlays.png", dpi=135)
    plt.close(fig)


def score_volume(rows, metric):
    field = np.empty((21, 21, 11))
    raw = np.array([float(row[metric]) for row in rows])
    mass = np.exp(ALPHA * (raw - raw.max()))
    mass /= mass.sum()
    for idx, row in enumerate(rows):
        cell = tuple(int(row[f"_axis{i}"]) for i in range(3))
        field[cell] = raw[idx] if metric == "native" else mass[idx]
    return field


def plot_score_surfaces(plt, path, grid, all_modes, recall):
    samples = ["H309"]
    normal = next((row["sample_id"] for row in recall if row["group"] == "B"), None)
    if normal:
        samples.append(normal)
    if "H309" not in grid:
        return
    pose_scope = {row["sample_id"]: row for row in recall if row["metric"] == "native"}
    fig, axes = plt.subplots(len(samples), 2, figsize=(11.3, 5.0 * len(samples)), squeeze=False,
                             constrained_layout=True)
    for r, sample in enumerate(samples):
        rows = grid[sample]
        indices = {row["index"]: row for row in rows}
        for c, metric in enumerate(METRICS):
            axis = axes[r, c]
            values = score_volume(rows, metric)
            peak = next((row for row in all_modes if row["sample_id"] == sample and row["metric"] == metric
                         and row["rank"] == "1"), None)
            peak_cell = indices[peak["index"]] if peak else max(rows, key=lambda row: float(row[metric]))
            angle = int(peak_cell["_axis2"])
            im = axis.imshow(values[:, :, angle].T, origin="lower", extent=(-2.1, 2.1, -2.1, 2.1),
                             aspect="equal", cmap="viridis", interpolation="nearest")
            mode_class = peak["mode_class"] if peak else "NO_VALID_MODE (ungated slice)"
            axis.set(xlabel="Heading-local X (m)", ylabel="Heading-local Y (m)",
                     title=f"{sample} | {metric.title()} | yaw {float(peak_cell['dyaw']):+.0f}°\n{mode_class}")
            axis.scatter(float(peak_cell["dx"]), float(peak_cell["dy"]), c="white", marker="x", s=45, label="peak")
            row = pose_scope.get(sample)
            if row and row.get("reference_valid"):
                axis.scatter(float(row["reference_dx"]), float(row["reference_dy"]), marker="*", c="#ff7426",
                             s=75, label="reference XY projection")
            axis.legend(loc="lower left", fontsize=8)
            fig.colorbar(im, ax=axis, label="score per source point" if metric == "native" else "Park grid probability mass")
    fig.suptitle("Fixed-heading-plane slices | Z, roll, and pitch held fixed at historical NDT solution")
    fig.savefig(path / "xy_native_park_normal_vs_failure.png", dpi=145)
    plt.close(fig)

    rows = grid["H309"]
    indices = {row["index"]: row for row in rows}
    fig, axes = plt.subplots(2, 3, figsize=(13.2, 6.4), squeeze=False, constrained_layout=True)
    for row_id, metric in enumerate(METRICS):
        volume = score_volume(rows, metric)
        peak = next(
            (row for row in all_modes if row["sample_id"] == "H309" and row["metric"] == metric and row["rank"] == "1"), None
        )
        selected = indices[peak["index"]] if peak else max(rows, key=lambda row: float(row[metric]))
        by_coord = [int(selected[f"_axis{i}"]) for i in range(3)]
        for d, (offsets, name, axis) in enumerate(zip((np.linspace(-2, 2, 21), np.linspace(-2, 2, 21), range(-5, 6)),
                                                    ("X offset (m)", "Y offset (m)", "Yaw offset (deg)"), axes[row_id])):
            sliced = list(by_coord)
            sliced[d] = slice(None)
            section = volume[tuple(sliced)]
            axis.plot(offsets, section, marker=".", color=("#007c91" if metric == "native" else "#a65300"))
            axis.axvline(offsets[by_coord[d]], ls="--", color="gray", lw=1)
            axis.set(xlabel=name, ylabel="Score / point" if metric == "native" else "Probability mass")
            axis.set_title(f"{metric.title()} through pose {selected['index']}")
    fig.suptitle("raw309 | each profile fixes the other two searched coordinates at the marked peak")
    fig.savefig(path / "raw309_scoring_profiles.png", dpi=145)
    plt.close(fig)


def render_coverage(plt, path, rows):
    keys = list(dict.fromkeys(row["sample_id"] for row in rows))
    summaries = {(row["sample_id"], row["metric"]): row for row in rows}
    values = np.full((7, len(keys)), np.nan)
    for col, sample in enumerate(keys):
        for group, metric in enumerate(METRICS):
            summary = summaries[(sample, metric)]
            if summary["reference_valid"]:
                values[0, col] = float(summary["original_ndt_correct"])
                for offset, k in enumerate(K_VALUES):
                    values[1 + 3 * group + offset, col] = float(summary[f"top{k}_correct"])
    fig, axis = plt.subplots(figsize=(12, 4), layout="constrained")
    from matplotlib.colors import ListedColormap
    cm = ListedColormap(["#f3c1ae", "#b9e3d0"])
    cm.set_bad("#dadada")
    axis.imshow(values, vmin=0, vmax=1, cmap=cm, interpolation="nearest", aspect="auto")
    for row, indicators in enumerate(values):
        for col, value in enumerate(indicators):
            axis.text(col, row, "—" if np.isnan(value) else ("hit" if value else "miss"),
                      ha="center", va="center", fontsize=8)
    axis.set_xticks(range(len(keys)), keys, rotation=35)
    axis.set_yticks(range(7), ["Original NDT", *[f"Native Top-{k}" for k in K_VALUES],
                              *[f"Park Top-{k}" for k in K_VALUES]])
    axis.set_title("Qualification recall | joint tolerance 0.15 m and 2°")
    fig.savefig(path / "candidate_reachability.png", dpi=150)
    plt.close(fig)


def render_microstructure(plt, path, scales, fit_rows):
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.6), constrained_layout=True)
    source = [row for row in scales if row["sample_id"] == "H309"]
    seen = set()
    kind_labels = {
        "grid_best_absolute": "grid vs reference",
        "refined_absolute": "fit vs reference",
        "park_local_spread_at_native_best": "local Park spread at native peak",
        "park_local_spread_at_park_best": "local Park spread at Park peak",
    }
    for row in source:
        name = (row["kind"], row["metric"], row["index"])
        if name in seen or row["kind"] not in kind_labels:
            continue
        seen.add(name)
        subset = [r for r in source if (r["kind"], r["metric"], r["index"]) == name]
        axes[0].plot([r["range_m"] for r in subset], [r["conservative_sum_cm"] for r in subset],
                     marker=".", label=f"{kind_labels[row['kind']]} ({row['metric']}:{row['index']})")
    axes[0].axhline(5, ls="--", c="#c74f0b", lw=1, label="5 cm comparison scale")
    axes[0].set(xlabel="Range (m)", ylabel="Translation + rotational displacement (cm)", title="raw309")
    axes[0].legend(fontsize=7)
    all_refits = [row for row in fit_rows if "fit_translation_error_m" in row]
    for axis, (mode, color) in zip(axes[1:], (("translation", "#007c91"), ("rotation", "#b64c22"))):
        unit = "m" if mode == "translation" else "deg"
        field = f"{mode}_error_{unit}"
        values = [(row[f"seed_{field}"], row[f"fit_{field}"]) for row in all_refits]
        if values:
            pareto = np.array(values)
            shown = np.maximum(pareto, 1e-5)
            axis.scatter(shown[:, 0], shown[:, 1], c=color, s=20, alpha=0.72)
            bounds = [shown.min(), shown.max()]
            axis.plot(bounds, bounds, c="gray", lw=1)
        axis.set(xscale="log", yscale="log", xlabel=f"Seed error ({unit})",
                 ylabel=f"Refined terminal error ({unit})", title=f"{mode.title()} before and after refinement")
    fig.savefig(path / "pose_scale_and_refinement.png", dpi=150)
    plt.close(fig)


def recall_aggregate(rows):
    keys = ("original_ndt_correct", "any_correct_grid_pose", "any_correct_supported_grid_pose",
            "top1_correct", "top3_correct", "top5_correct")
    for metric in METRICS:
        for distance in DISTANCE_LIMITS:
            for angle in ANGLE_LIMITS:
                subset = [
                    row for row in rows
                    if row["metric"] == metric and row["translation_tol_m"] == distance and row["rotation_tol_deg"] == angle
                ]
                for group in ("ALL", "A", "B", "C"):
                    cases = [row for row in subset if group == "ALL" or row["group"] == group]
                    count_valid = sum(bool(row["reference_valid"]) for row in cases)
                    summary = dict(
                        metric=metric, group=group, translation_tol_m=distance, rotation_tol_deg=angle,
                        valid_reference_count=count_valid, planned_count=len(cases),
                    )
                    for key in keys:
                        count = sum(row.get(key, False) for row in cases)
                        summary[key + "_count"] = count
                        summary[key + "_recall_valid"] = safe_relative(count, count_valid)
                    yield summary


def terminal_pair_evidence(grid, modes, fit_rows):
    from peaks import node_index, widest_paths

    by_index = {row["index"]: row for row in grid}
    mode_map = {(row["metric"], row["index"]): row for row in modes if bool_cell(row["retained"])}
    pair_rows = []
    supported_modes = {}
    path_cache = {}
    for n, first in enumerate(fit_rows):
        for second in fit_rows[:n]:
            shift, turn = (float(x) for x in pose_errors(transform(first), transform(second)))
            qualified = first["qualified_terminal"] and second["qualified_terminal"]
            separated = shift > 0.30 or turn > 3.0
            row = dict(
                sample_id=first["sample_id"], index_a=first["index"], index_b=second["index"],
                terminal_translation_m=shift, terminal_rotation_deg=turn,
                both_qualified=qualified, terminals_separated=separated,
                both_terminal_native_rel=float(min(float(first["native"]), float(second["native"])) /
                                               max(1e-15, float(first["native"]), float(second["native"]))),
            )
            for metric in METRICS:
                a = mode_map.get((metric, first["index"]))
                b = mode_map.get((metric, second["index"]))
                row[f"{metric}_peak_pair"] = bool(a and b and not bool_cell(a["boundary"]) and not bool_cell(b["boundary"]))
                row[f"{metric}_independent_grid_pair"] = False
                row[f"{metric}_verified_competitive_terminals"] = False
                if not row[f"{metric}_peak_pair"]:
                    continue
                a_rel, b_rel = float(a["relative_score"]), float(b["relative_score"])
                first_node = node_index(by_index[a["index"]])
                second_node = node_index(by_index[b["index"]])
                if metric not in supported_modes:
                    valid = [g for g in grid if support(g, metric) >= 0.5 and np.isfinite(float(g[metric]))]
                    best = max(float(g[metric]) for g in valid)
                    supported_modes[metric] = {
                        node_index(g): float(g[metric]) / best if metric == "native" else
                        float(np.exp(ALPHA * (float(g[metric]) - best))) for g in valid
                    }
                key = (metric, first_node)
                if key not in path_cache:
                    path_cache[key] = widest_paths(supported_modes[metric], first_node)
                saddle = path_cache[key].get(second_node)
                row[f"{metric}_saddle_relative_to_lower_peak"] = None if saddle is None else saddle / min(a_rel, b_rel)
                independent = saddle is not None and min(a_rel, b_rel) >= 0.95 - 1e-12 and saddle <= 0.95 * min(a_rel, b_rel) + 1e-12
                row[f"{metric}_independent_grid_pair"] = independent
                row[f"{metric}_verified_competitive_terminals"] = bool(qualified and separated and independent)
            pair_rows.append(row)
    return pair_rows


def curvature_summary(path):
    rows = []
    fields = [f"h{r}{c}" for r in range(6) for c in range(6)]
    # Native coordinates use translation then Euler angles in radians.
    # Express angular deviations as displacement at fixed scale 5 m.
    scale = np.diag([1, 1, 1, 0.2, 0.2, 0.2])
    for source in load_csv(path):
        h = np.array([float(source[field]) for field in fields]).reshape(6, 6)
        scaled_information = -scale @ ((h + h.T) / 2) @ scale
        eigen = np.linalg.eigvalsh(scaled_information) if np.isfinite(h).all() else np.full(6, np.nan)
        rows.append(dict(
            sample_id=source["sample_id"], index=source["index"], stage=source["stage"],
            **{f"negative_hessian_scaled_eigen_{i}": str(value) if np.isfinite(value) else "" for i, value in enumerate(eigen)},
            spectral_symmetry_residual=float(np.linalg.norm(h - h.T)),
            positive_definite=bool((eigen > 0).all()),
            rotation_length_scale_m=5.0,
        ))
    return rows


def make_summary(primary, order, pairs, refits):
    summary = {"recall": {}, "sample_count": len({row["sample_id"] for row in primary})}
    for metric in METRICS:
        rows = [row for row in primary if row["metric"] == metric]
        valid = [row for row in rows if row["reference_valid"]]
        summary["recall"][metric] = dict(
            references=len(valid), planned=len(rows),
            original=sum(row["original_ndt_correct"] for row in valid),
            all_grid=sum(row["any_correct_grid_pose"] for row in valid),
            supported_grid=sum(row["any_correct_supported_grid_pose"] for row in valid),
            modes=dict(Counter(row["mode_class"] for row in rows)),
            failures=dict(Counter(row["primary_failure_class"] for row in rows)),
            verified_competitive_native_fit_samples=sorted({
                row["sample_id"] for row in pairs if row.get(f"{metric}_verified_competitive_terminals")
            }),
            **{f"top{k}": sum(row[f"top{k}_correct"] for row in valid) for k in K_VALUES},
        )
    summary["raw309"] = [row for row in primary if row["sample_id"] == "H309"]
    summary["raw309_ranking"] = next((row for row in order if row["sample_id"] == "H309"), None)
    summary["qualified_terminals"] = sum(row["qualified_terminal"] for row in refits)
    summary["refinement_runs"] = len(refits)
    summary["terminal_correct_015m_2deg"] = sum(row.get("fit_correct_015m_2deg", False) for row in refits)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--online", required=True, type=Path)
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--grid", required=True, type=Path)
    parser.add_argument("--modes", required=True, type=Path)
    parser.add_argument("--fits", required=True, type=Path)
    parser.add_argument("--reference", required=True, type=Path)
    parser.add_argument("--seal", required=True, type=Path)
    parser.add_argument("--curvature", type=Path)
    parser.add_argument("--out", required=True, type=Path)
    args = parser.parse_args()
    required = [args.online, args.grid, args.modes, args.fits]
    if args.curvature:
        required.append(args.curvature)
    verify_seal(args.seal, required, args.manifest)

    manifest = {row["sample_id"]: row for row in load_csv(args.manifest)}
    frames = read_inputs(args.online)
    grid = read_grid(args.grid)
    if set(frames) != set(grid) or set(grid) != set(manifest):
        raise ValueError("online, manifest, and grid sample inventories differ")
    for name, source in (("map_sha256", "map_path"), ("prepared_pcd_sha256", "scan_path")):
        verified = {}
        for sample, frame in frames.items():
            expected = manifest[sample][name]
            asset = getattr(frame, source)
            checksum = verified.get(asset)
            if checksum is None:
                checksum = digest(asset)
                verified[asset] = checksum
            if checksum != expected:
                raise ValueError(f"{sample}: asset mutated since manifest freeze: {asset}")

    references = read_reference(args.reference)
    if unknown := set(references) - set(frames):
        raise ValueError(f"references contain unexpected IDs: {sorted(unknown)}")
    all_modes = load_csv(args.modes)
    all_fits = load_csv(args.fits)
    modes_by_sample = defaultdict(list)
    fits_by_sample = defaultdict(list)
    for row in all_modes:
        modes_by_sample[row["sample_id"]].append(row)
    for row in all_fits:
        fits_by_sample[row["sample_id"]].append(row)
    if set(modes_by_sample) != set(frames) or set(fits_by_sample) - set(frames):
        raise ValueError("mode or fit sample inventories differ")

    recall, order, joined_modes, refits, scales, pairs = [], [], [], [], [], []
    for sample, frame in frames.items():
        partial = evaluate_sample(
            sample, grid[sample], modes_by_sample[sample], fits_by_sample[sample],
            frame, references.get(sample), manifest[sample],
        )
        recall_rows, ranking_row, modes, fitted, scale = partial
        recall.extend(recall_rows)
        order.append(ranking_row)
        joined_modes.extend(modes)
        refits.extend(fitted)
        scales.extend(scale)
        pairs.extend(terminal_pair_evidence(grid[sample], modes_by_sample[sample], fitted))

    args.out.mkdir(parents=True, exist_ok=False)
    write_csv(args.out / "candidate_recall.csv", recall)
    write_csv(args.out / "candidate_recall_summary.csv", list(recall_aggregate(recall)))
    write_csv(args.out / "score_ranking_comparison.csv", order)
    write_csv(args.out / "candidate_mode_offline_errors.csv", joined_modes)
    if refits:
        write_csv(args.out / "refinement_offline_errors.csv", refits)
    if scales:
        write_csv(args.out / "candidate_microstructure_scale.csv", scales)
    if pairs:
        write_csv(args.out / "native_terminal_mode_evidence.csv", pairs)
    if args.curvature:
        write_csv(args.out / "candidate_curvature_summary.csv", curvature_summary(args.curvature))

    primary = [row for row in recall if row["translation_tol_m"] == 0.15 and row["rotation_tol_deg"] == 2.0]
    summary = make_summary(primary, order, pairs, refits)
    (args.out / "evaluation_summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figures = args.out / "figures"
    figures.mkdir()
    if "H309" in frames:
        visualize_overlays(plt, figures, frames["H309"], references.get("H309"), grid["H309"],
                           modes_by_sample["H309"], fits_by_sample["H309"])
        plot_score_surfaces(plt, figures, grid, all_modes, primary)
    render_coverage(plt, figures, primary)
    render_microstructure(plt, figures, scales, refits)


if __name__ == "__main__":
    main()
