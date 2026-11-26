"""Evaluate bounded Q2 search from sealed output, with references read offline."""

import argparse
import csv
import json
import math
import sys
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np

Q1_DOCS = Path(__file__).resolve().parents[1] / "p64_r2_q1_candidate_search"
sys.path.insert(0, str(Q1_DOCS))
from evaluate import (
    bool_cell,
    digest,
    load_csv,
    pose_errors,
    read_inputs,
    read_reference,
    transform,
    write_csv,
)

METHODS = ("S0", "S1", "S2", "S3")
METRICS = ("native", "park")
POLICIES = ("C0", "C1", "C2", "C3")
PRIMARY = (0.15, 2.0)
TOLERANCES = ((0.10, 1.0), PRIMARY, (0.20, 3.0), (0.50, 5.0))
TOP_K = (1, 3, 5)
MODEL_RANGE_M = (2, 5, 10, 20)


def validate_seal(args):
    receipt = json.loads(args.seal.read_text())
    matched = {}
    for filename, expected in receipt["files"].items():
        path = args.seal.parent / filename
        if digest(path) != expected:
            raise ValueError(f"changed file in sealed Q2 receipt: {path}")
        matched[path.resolve()] = expected
    requested = (args.online, args.grid, args.retained, args.fits, args.runtime)
    if missing := {asset.resolve() for asset in requested} - matched.keys():
        raise ValueError(f"unsealed Q2 inputs: {sorted(map(str, missing))}")
    if receipt.get("manifest_sha256") and digest(args.manifest) != receipt["manifest_sha256"]:
        raise ValueError("sample manifest hash differs from seal")
    # Reused Q1 modes and fits also retain Q1's cryptographic provenance.
    q1_seal = json.loads((args.q1_run / "online_seal.json").read_text())
    if digest(args.manifest) != q1_seal["manifest_sha256"]:
        raise ValueError("sample manifest changed from Q1")
    if digest(args.online) != q1_seal["files"]["input/online.tsv"]:
        raise ValueError("online search input changed from Q1")
    reused = ("official/grid.csv", "official/modes.csv", "official/fit.csv", "fit/point_scores.csv")
    for name in reused:
        checksum = q1_seal["files"][name]
        if digest(args.q1_run / name) != checksum:
            raise ValueError(f"Q1 reused evidence changed: {name}")


def canon_metric(name):
    normalized = name.lower()
    metric = {"native_diagnostic": "native", "park_diagnostic": "park"}.get(normalized, normalized)
    if metric not in METRICS:
        raise ValueError(f"unexpected score metric: {name}")
    return metric


def metric_support(row, metric):
    field = "park_associated" if metric == "park" else "associated"
    return int(row[field]) / int(row["total"])


def group_rows(rows, keys):
    groups = defaultdict(list)
    for row in rows:
        groups[tuple(row[key] for key in keys)].append(row)
    return groups


def read_grid_sources(path, q1_dir):
    rows = load_csv(path)
    if not rows:
        raise ValueError("grid file contains no scored rows")
    baseline = load_csv(q1_dir / "official/grid.csv")
    copied_s0 = {(row["sample_id"], row["index"]): row for row in rows if row["method"] == "S0"}
    if copied_s0:
        compare = ("native", "park", "associated", "park_associated", "total",
                   *(f"T{r}{c}" for r in range(4) for c in range(4)))
        if len(copied_s0) != len(baseline):
            raise ValueError("reused S0 output inventory differs from Q1")
        for old in baseline:
            actual = copied_s0[old["sample_id"], old["index"]]
            if any(float(actual[key]) != float(old[key]) for key in compare):
                raise ValueError(f"S0 export changed from Q1 for {old['sample_id']}:{old['index']}")
    else:
        for old in baseline:
            boundary = int(old["ix"]) in (0, 20) or int(old["iy"]) in (0, 20) or int(old["ia"]) in (0, 10)
            rows.append({**old, "method": "S0", "boundary": str(int(boundary)), "center": "ndt",
                         "stage": "base", "dz": "0", "droll": "0", "dpitch": "0"})
    grouped = group_rows(rows, ("sample_id", "method"))
    for key, cells in grouped.items():
        if len(cells) != len({cell["index"] for cell in cells}) or len(cells) > 4851:
            raise ValueError(f"{key}: repeated output grid indices or fixed scoring budget exceeded")
        if any(int(cell["total"]) <= 0 for cell in cells):
            raise ValueError(f"{key}: invalid source point count")
    return grouped


def read_retention(path, q1_dir):
    rows = load_csv(path)
    if any(row["method"] == "S0" and row["policy"] == "C0" for row in rows):
        raise ValueError("S0 C0 must use Q1's retained candidates directly")
    for row in load_csv(q1_dir / "official/modes.csv"):
        rows.append({**row, "method": "S0", "policy": "C0", "peak_class": row["mode_class"]})
    return group_rows(rows, ("sample_id", "method", "metric", "policy"))


def q1_refit_park(q1_dir):
    sums = defaultdict(float)
    hits = Counter()
    counts = Counter()
    with (q1_dir / "fit/point_scores.csv").open(newline="") as stream:
        for row in csv.DictReader(stream):
            if row["pose_id"].startswith("fit_"):
                key = row["sample_id"], row["pose_id"][4:]
                sums[key] += float(row["park_log"])
                hits[key] += int(row["park_associated"])
                counts[key] += 1
    return sums, hits, counts


def read_fits(path, q1_dir, grids):
    rows = load_csv(path)
    for row in rows:
        row["origin"] = "Q2_NEW"
    sums, hits, counts = q1_refit_park(q1_dir)
    indexes_q1 = {
        key[0]: {cell["index"]: cell for cell in cells}
        for key, cells in grids.items() if key[1] == "S0"
    }
    for old in load_csv(q1_dir / "official/fit.csv"):
        sample = old["sample_id"]
        index = old["index"]
        key = sample, index
        if counts[key] != int(old["total"]):
            raise ValueError(f"Q1 terminal per-point support miscount for {key}")
        seed = indexes_q1[sample][index]
        movement, turn = pose_errors(transform(old), transform(seed))
        rows.append({
            **old, "method": "S0", "metric": canon_metric(old["seed_kind"]), "policy": "C0",
            "drift_translation_m": float(movement), "drift_rotation_deg": float(turn),
            "mode_preserved": movement <= 0.35 and turn <= 4.0,
            "park": sums[key], "park_associated": hits[key], "origin": "Q1_REUSED",
        })
    return group_rows(rows, ("sample_id", "method"))


def with_errors(row, reference):
    if reference is None:
        return {**row, "ref_translation_m": None, "ref_rotation_deg": None}
    travel, turn = (float(number) for number in pose_errors(transform(row), reference))
    return {**row, "ref_translation_m": travel, "ref_rotation_deg": turn}


def qualify_fit(row, reference):
    row = with_errors(row, reference)
    qualified = (bool_cell(row["reported_converged"]) and int(row["iterations"]) > 0
                 and metric_support(row, "native") >= 0.50)
    movement_ok = float(row["drift_translation_m"]) <= 0.35 + 1e-9 and float(row["drift_rotation_deg"]) <= 4.0 + 1e-9
    reported_guard = movement_ok and (qualified if row["origin"] == "Q2_NEW" else True)
    if reported_guard != bool_cell(row["mode_preserved"]):
        raise ValueError(f"{row['sample_id']} {row['method']} {row['index']}: drift guard mismatch")
    row["qualified_native_fit"] = qualified
    row["guarded_native_online_flag"] = bool_cell(row["mode_preserved"])
    row["pose_drift_only_preserved"] = movement_ok
    for metric in METRICS:
        row[f"accepted_{metric}"] = qualified and movement_ok and metric_support(row, metric) >= 0.50
    return row


def good(row, tol=PRIMARY):
    return (
        row.get("ref_translation_m") is not None
        and row["ref_translation_m"] <= tol[0] + 1e-9
        and row["ref_rotation_deg"] <= tol[1] + 1e-8
    )


def best_joint(rows, tol=PRIMARY):
    valid = [row for row in rows if row.get("ref_translation_m") is not None]
    if valid:
        return min(valid, key=lambda row: (
            max(row["ref_translation_m"] / tol[0], row["ref_rotation_deg"] / tol[1]),
            row["ref_translation_m"], row["ref_rotation_deg"], int(row["index"]),
        ))
    return None


def select_rank(rows, metric):
    return sorted(rows, key=lambda row: (-float(row[metric]), int(row["index"])))


def qualified_modes(mode_rows, cells):
    kept = sorted((row for row in mode_rows if bool_cell(row["retained"])), key=lambda row: int(row["rank"]))
    return [(row, cells[row["index"]]) for row in kept]


def acceptable(points, tol):
    return any(good(row, tol) for row in points)


def uniform_cell_coverage(grid, reference):
    matrices = np.stack([transform(row) for row in grid])
    if reference is None:
        return {row["index"]: {**row, "ref_translation_m": None, "ref_rotation_deg": None} for row in grid}
    t, r = pose_errors(matrices, reference)
    return {row["index"]: {**row, "ref_translation_m": float(t[idx]), "ref_rotation_deg": float(r[idx])}
            for idx, row in enumerate(grid)}


def distinct_terminal_count(final):
    representatives = []
    for row in final:
        pose = transform(row)
        distinct = True
        for previous in representatives:
            distance, angle = pose_errors(pose, previous)
            if distance <= 0.30 and angle <= 3.0:
                distinct = False
                break
        if distinct:
            representatives.append(pose)
    return len(representatives)


def evaluate_selection(cells, mode_rows, all_fits, sample, method, metric, policy):
    kept_pairs = qualified_modes(mode_rows, cells)
    retained = [cell for _, cell in kept_pairs]
    possible = {row["index"] for row in retained}
    hits = [fit for fit in all_fits if fit["index"] in possible]
    approved = [fit for fit in hits if fit[f"accepted_{metric}"]]
    refined = [fit for fit in hits if fit["qualified_native_fit"]]
    final = retained
    if policy == "C3":
        successful = {fit["index"]: fit for fit in approved}
        final = select_rank([successful.get(row["index"], row) for row in retained], metric)
    ranked_records = []
    final_ranks = {row["index"]: rank for rank, row in enumerate(final, 1)}
    final_cells = {row["index"]: row for row in final}
    for mode, original in kept_pairs:
        final_pose = final_cells[original["index"]]
        record = {
            "sample_id": sample, "method": method, "metric": metric, "policy": policy,
            "index": original["index"], "coarse_or_grid_rank": mode["rank"],
            "coarse_or_grid_score": original[metric], "boundary": mode["boundary"],
            "coarse_translation_error_m": original["ref_translation_m"],
            "coarse_rotation_error_deg": original["ref_rotation_deg"],
            "coarse_correct": good(original), "final_rank": final_ranks[original["index"]],
            "final_score": final_pose[metric], "final_support": metric_support(final_pose, metric),
            "final_pose_source": final_pose.get("origin", "grid"),
            "final_translation_error_m": final_pose["ref_translation_m"],
            "final_rotation_error_deg": final_pose["ref_rotation_deg"],
            "final_correct": good(final_pose),
        }
        ranked_records.append(record)
    return retained, refined, approved, final, ranked_records


def evaluate_matrix(grids, modes, fits, references, manifest):
    coverage = []
    ranks = []
    refit_diagnostics = []
    evaluated_grid = {}
    for (sample, method), score_rows in grids.items():
        reference = references.get(sample)
        cells = uniform_cell_coverage(score_rows, reference)
        evaluated_grid[sample, method] = cells
        terminals = [qualify_fit(row, reference) for row in fits[(sample, method)]]
        new_count = sum(fit["origin"] == "Q2_NEW" for fit in terminals)
        if new_count > (2 if method == "S0" else 4):
            raise ValueError(f"{sample}/{method}: new alignment trial budget exceeded")
        for fit in terminals:
            seed = cells[fit["index"]]
            fit.update(seed_translation_error_m=seed["ref_translation_m"], seed_rotation_error_deg=seed["ref_rotation_deg"],
                       primary_seed_correct=good(seed), primary_terminal_correct=good(fit),
                       boundary_seed=bool_cell(seed["boundary"]),
                       native_score_change=float(fit["native"]) - float(seed["native"]),
                       park_score_change=float(fit["park"]) - float(seed["park"]))
        refit_diagnostics.extend(terminals)
        for metric in METRICS:
            supported = [row for row in cells.values() if metric_support(row, metric) >= 0.50]
            if metric == "native":
                supported = [row for row in supported if float(row[metric]) > 0]
            highest = select_rank(supported, metric)[0] if supported else None
            for policy in POLICIES:
                key = sample, method, metric, policy
                candidate_modes = modes[key]
                retained, refined, approved, final, ranking = evaluate_selection(cells, candidate_modes, terminals, *key)
                ranks.extend(ranking)
                for tol in TOLERANCES:
                    nearest = best_joint(list(cells.values()), tol)
                    supports_nearest = best_joint(supported, tol)
                    raw_peak_hits = [row for row in candidate_modes
                                     if row["index"] in cells and good(cells[row["index"]], tol)]
                    data = {
                        "sample_id": sample, "group": manifest[sample]["group"], "protocol": manifest[sample]["protocol"],
                        "method": method, "metric": metric, "policy": policy, "reference_valid": reference is not None,
                        "translation_tol_m": tol[0], "rotation_tol_deg": tol[1], "unique_grid_n": len(cells),
                        "supported_grid_n": len(supported), "retained_n": len(retained), "alignment_trials": len(terminals),
                        "new_alignments": new_count, "refined_retained_n": len(refined), "accepted_refinements_n": len(approved),
                        "grid_any_correct": acceptable(cells.values(), tol),
                        "supported_grid_any_correct": acceptable(supported, tol),
                        "raw_peak_any_correct": bool(raw_peak_hits),
                        "boundary_retained_any_correct": any(
                            good(cells[row["index"]], tol) and bool_cell(row["boundary"])
                            for row in candidate_modes if bool_cell(row["retained"])
                        ),
                        "retained_any_correct": acceptable(retained, tol),
                        "native_fit_trial_any_correct": acceptable([r for r in terminals if r["qualified_native_fit"]], tol),
                        "refined_retained_any_correct": acceptable(refined, tol),
                        "accepted_refined_any_correct": acceptable(approved, tol),
                        "best_grid_index": None if highest is None else highest["index"],
                        "nearest_grid_index": None if nearest is None else nearest["index"],
                        "nearest_grid_translation_m": None if nearest is None else nearest["ref_translation_m"],
                        "nearest_grid_rotation_deg": None if nearest is None else nearest["ref_rotation_deg"],
                        "nearest_supported_grid_index": None if supports_nearest is None else supports_nearest["index"],
                        "final_winner_index": None if not final else final[0]["index"],
                        "final_winner_pose_source": None if not final else final[0].get("origin", "grid"),
                        "final_winner_translation_m": None if not final else final[0]["ref_translation_m"],
                        "final_winner_rotation_deg": None if not final else final[0]["ref_rotation_deg"],
                    }
                    data["final_distinct_modes_n"] = distinct_terminal_count(final)
                    for k in TOP_K:
                        data[f"retained_top{k}_correct"] = acceptable(retained[:k], tol)
                        data[f"final_top{k}_correct"] = acceptable(final[:k], tol)
                    coverage.append(data)
    return coverage, ranks, refit_diagnostics, evaluated_grid


def aggregate(coverage):
    fields = [
        "grid_any_correct", "supported_grid_any_correct", "retained_any_correct", "native_fit_trial_any_correct",
        "refined_retained_any_correct", "accepted_refined_any_correct", *[f"retained_top{k}_correct" for k in TOP_K],
        *[f"final_top{k}_correct" for k in TOP_K],
    ]
    rows = []
    grouped = group_rows(coverage, ("method", "metric", "policy", "translation_tol_m", "rotation_tol_deg"))
    for (method, metric, policy, t, r), records in grouped.items():
        for group in ("ALL", "A", "B", "C"):
            subset = [row for row in records if group == "ALL" or row["group"] == group]
            available = [row for row in subset if row["reference_valid"]]
            result = {"method": method, "metric": metric, "policy": policy, "group": group,
                          "translation_tol_m": t, "rotation_tol_deg": r, "planned_n": len(subset), "reference_n": len(available)}
            for field in fields:
                hits = sum(row[field] for row in available)
                result[field + "_n"] = hits
                result[field + "_rate"] = hits / len(available) if available else None
            rows.append(result)
    return rows


def compare_retention(primary):
    base = {(r["sample_id"], r["method"], r["metric"]): r for r in primary if r["policy"] == "C0"}
    rows = []
    for row in primary:
        if row["policy"] == "C0":
            continue
        prior = base[row["sample_id"], row["method"], row["metric"]]
        item = {k: row[k] for k in ("sample_id", "method", "metric", "policy", "group", "reference_valid")}
        for field in ("retained_any_correct", "retained_top1_correct", "final_top1_correct", "final_top5_correct"):
            item[f"C0_{field}"] = prior[field]
            item[field] = row[field]
            item[f"change_{field}"] = int(row[field]) - int(prior[field]) if row["reference_valid"] else None
        rows.append(item)
    return rows


def summarize_runtime(path):
    measured = load_csv(path)
    totals = ("score_calls", "native_calls", "park_calls", "transform_calls", "native_queries", "park_queries",
              "align_calls", "unique_poses", "proposals", "duplicates", "postfit_score_calls",
              "postfit_native_queries", "postfit_park_queries")
    timed = [field for field in measured[0] if field.endswith("_s")] if measured else []
    result = []
    for method in METHODS:
        rows = [row for row in measured if row["method"] == method]
        if not rows:
            continue
        summary = {"method": method, "frame_rows": len(rows)}
        for field in totals:
            if field in rows[0]:
                values = [int(row[field]) for row in rows]
                summary[f"{field}_total"] = sum(values)
                summary[f"{field}_max"] = max(values)
        for field in timed:
            values = [float(row[field]) for row in rows]
            summary[f"{field}_total"] = sum(values)
            summary[f"{field}_p50"] = float(np.percentile(values, 50))
            summary[f"{field}_p95"] = float(np.percentile(values, 95))
            summary[f"{field}_max"] = max(values)
        result.append(summary)
    return measured, result


def record_raw309(evaluated_grid, frames, references, primary):
    focus = "H309"
    if focus not in references:
        return []
    original, prediction = frames[focus].original, frames[focus].prediction
    reference = references[focus]
    centers = []
    for name, pose in (("pred", prediction), ("ndt", original)):
        e_t, e_r = (float(x) for x in pose_errors(pose, reference))
        delta = reference[:3, :3] @ pose[:3, :3].T
        yaw = math.degrees(math.atan2(delta[1, 0] - delta[0, 1], delta[0, 0] + delta[1, 1]))
        centers.append({
            "kind": "fixed_input_center", "center": name, "translation_error_m": e_t,
            "rotation_error_deg": e_r, "relative_reference_yaw_deg": yaw,
        })
    for method in METHODS:
        cells = evaluated_grid[focus, method]
        for source in sorted({cell["center"] for cell in cells.values()}):
            subset = [cell for cell in cells.values() if cell["center"] == source]
            closest = best_joint(subset)
            centers.append({
                "kind": "measured_grid_coverage", "method": method, "center": source,
                "score_pose_n": len(subset), "correct_pose_n": sum(good(cell) for cell in subset),
                "closest_grid_index": closest["index"], "translation_error_m": closest["ref_translation_m"],
                "rotation_error_deg": closest["ref_rotation_deg"], "closest_stage": closest.get("stage", ""),
            })
    for row in primary:
        if row["sample_id"] == focus and row["policy"] == ("C0" if row["method"] == "S0" else "C3"):
            centers.append({
                "kind": "selected_final", "method": row["method"], "metric": row["metric"],
                "policy": row["policy"], "winner_index": row["final_winner_index"],
                "translation_error_m": row["final_winner_translation_m"], "rotation_error_deg": row["final_winner_rotation_deg"],
            })
    return centers


def microstructure_records(primary, refinements):
    rows = []
    for record in primary:
        if record["sample_id"] != "H309" or record["policy"] not in ("C0", "C3"):
            continue
        for stage in ("nearest_grid", "final_winner"):
            t = record[f"{stage}_translation_m"]
            r = record[f"{stage}_rotation_deg"]
            if t is None or r is None:
                continue
            for distance in MODEL_RANGE_M:
                rows.append({
                    "sample_id": "H309", "method": record["method"], "metric": record["metric"], "policy": record["policy"],
                    "stage": stage, "range_m": distance, "translation_error_m": t, "rotation_error_deg": r,
                    "rotational_displacement_cm": distance * math.radians(r) * 100.0,
                    "translation_plus_rotation_cm": (t + distance * math.radians(r)) * 100.0,
                })
    for fit in refinements:
        if fit["sample_id"] == "H309" and fit["ref_translation_m"] is not None:
            t, r = fit["ref_translation_m"], fit["ref_rotation_deg"]
            for distance in MODEL_RANGE_M:
                rows.append({
                    "sample_id": "H309", "method": fit["method"], "metric": fit["metric"], "policy": fit["policy"],
                    "stage": "refit_trial", "index": fit["index"], "range_m": distance, "translation_error_m": t, "rotation_error_deg": r,
                    "rotational_displacement_cm": distance * math.radians(r) * 100.0,
                    "translation_plus_rotation_cm": (t + distance * math.radians(r)) * 100.0,
                })
    return rows


def figure_coverage(plt, path, rows, manifest):
    from matplotlib.colors import ListedColormap

    sample_order = list(manifest)
    policies = {"S0": "C0", "S1": "C3", "S2": "C3", "S3": "C3"}
    subset = {(r["sample_id"], r["method"], r["metric"]): r for r in rows if r["policy"] == policies[r["method"]]}
    fig, axes = plt.subplots(2, 3, figsize=(15.6, 5.7), layout="constrained")
    fields = ("supported_grid_any_correct", "retained_any_correct", "final_top1_correct")
    labels = ("Supported grid coverage", "Retained coverage", "Final Top-1")
    for i, metric in enumerate(METRICS):
        for j, field in enumerate(fields):
            axis = axes[i, j]
            values = np.array([
                [float(subset[sample, method, metric][field]) if subset[sample, method, metric]["reference_valid"]
                 else np.nan for sample in sample_order] for method in METHODS
            ])
            colors = ListedColormap(["#f2c0a7", "#b0dbc5"])
            colors.set_bad("#dddddd")
            axis.imshow(values, aspect="auto", interpolation="none", cmap=colors, vmin=0, vmax=1)
            for yy, vector in enumerate(values):
                for xx, value in enumerate(vector):
                    axis.text(xx, yy, "–" if not np.isfinite(value) else ("●" if value else "×"),
                              ha="center", va="center", fontsize=8)
            axis.set_xticks(range(len(sample_order)), sample_order, rotation=50)
            axis.set_yticks(range(len(METHODS)), [f"{method}/{policies[method]}" for method in METHODS])
            axis.set_title(f"{metric.title()} · {labels[j]}")
    fig.suptitle("Twelve audited frames | 0.15 m and 2° joint success")
    fig.savefig(path / "grid_retained_final_coverage.png", dpi=145)
    plt.close(fig)


def figure_retention(plt, path, primary):
    fig, axes = plt.subplots(2, 2, figsize=(12.5, 7), layout="constrained")
    for (i, metric) in enumerate(METRICS):
        for j, field in enumerate(("retained_any_correct", "final_top1_correct")):
            axis = axes[i, j]
            x = np.arange(len(METHODS))
            for k, policy in enumerate(POLICIES):
                counts = [
                    sum(row[field] for row in primary if row["metric"] == metric and row["method"] == method
                        and row["policy"] == policy and row["reference_valid"])
                    for method in METHODS
                ]
                axis.bar(x + (k - 1.5) * 0.18, counts, 0.17, label=policy)
                for m, count in enumerate(counts):
                    axis.text(m + (k - 1.5) * 0.18, count + .15, str(count), ha="center", fontsize=7)
            axis.set(ylim=(0, 12.8), ylabel="Correct frames / 12", xticks=x, xticklabels=METHODS)
            axis.set_title(f"{metric.title()} · {'retained coverage' if j == 0 else 'final Top-1'}")
            axis.legend(loc="upper right", fontsize=8, ncols=2)
    fig.suptitle("Peak retention and guarded refinement | same input grids")
    fig.savefig(path / "retention_policy_comparison.png", dpi=145)
    plt.close(fig)


def figure_raw309(plt, path, evaluated_grid, frames, references):
    focus = "H309"
    reference = references.get(focus)
    if reference is None:
        return
    fig, axes = plt.subplots(2, 4, figsize=(15.5, 7.0), layout="constrained")
    frame = frames[focus]
    origin = reference[:3, 3]
    for col, method in enumerate(METHODS):
        cells = list(evaluated_grid[focus, method].values())
        axes[0, col].scatter(*((np.stack([transform(cell)[:3, 3] for cell in cells])[:, :2] - origin[:2]).T),
                             s=2, alpha=0.24, c="#565d67")
        axes[0, col].scatter(*(frame.prediction[:2, 3] - origin[:2]), c="#2166a5", marker="+", s=90, label="prediction")
        axes[0, col].scatter(*(frame.original[:2, 3] - origin[:2]), c="#d9790b", marker="x", s=65, label="native center")
        axes[0, col].scatter(0, 0, c="#990b31", marker="*", s=72, label="offline ref")
        axes[0, col].set(title=method, xlabel="Map X – ref (m)", ylabel="Map Y – ref (m)",
                         xlim=(-3.5, 3.5), ylim=(-3.5, 3.5), aspect="equal")
        if col == 0:
            axes[0, col].legend(fontsize=7, loc="upper right")
        values = np.array([[cell["ref_translation_m"], cell["ref_rotation_deg"]] for cell in cells])
        axes[1, col].scatter(*values.T, s=4, alpha=.22, color="#202d51")
        ok = np.array([good(cell) for cell in cells])
        if ok.any():
            axes[1, col].scatter(*values[ok].T, s=19, c="#cf6e12", label="joint hits")
        axes[1, col].fill_between([0, .15], 0, 2, color="green", alpha=.12)
        axes[1, col].axvline(.15, color="green", lw=.7)
        axes[1, col].axhline(2.0, color="green", lw=.7)
        axes[1, col].set(title=f"{int(ok.sum())}/{len(cells)} grid hits", xlabel="Translation error (m)",
                         ylabel="Rotation error (deg)", xlim=(-.02, 4.0), ylim=(-.3, 30))
    fig.suptitle("raw309 search centers, sampled positions, and joint pose reachability")
    fig.savefig(path / "raw309_center_and_grid_coverage.png", dpi=145)
    plt.close(fig)


def figure_refinement(plt, path, fits, grids):
    fig, axes = plt.subplots(1, 2, figsize=(11.8, 5.4), layout="constrained")
    for fit in fits:
        seed = grids[fit["sample_id"], fit["method"]][fit["index"]]
        accepted = fit["accepted_native"]
        if seed["ref_translation_m"] is None:
            continue
        for axis, field in zip(axes, ("ref_translation_m", "ref_rotation_deg")):
            axis.scatter(max(seed[field], 1e-5), max(fit[field], 1e-5),
                         marker="D" if fit["sample_id"] == "H309" else ".",
                         color="#1b7896" if accepted else "#cd5a27", alpha=.72,
                         s=32 if fit["sample_id"] == "H309" else 23)
    for axis, unit in zip(axes, ("m", "deg")):
        axis.set(xscale="log", yscale="log", xlabel=f"Grid seed error ({unit})",
                 ylabel=f"Native fit terminal error ({unit})")
        low = max(axis.get_xlim()[0], axis.get_ylim()[0])
        high = min(axis.get_xlim()[1], axis.get_ylim()[1])
        axis.plot([low, high], [low, high], color="#666666", lw=1)
    fig.suptitle("Refinement | blue = guarded Native acceptance, orange = rejected, diamond = raw309")
    fig.savefig(path / "refinement_before_after.png", dpi=145)
    plt.close(fig)


def figure_runtime(plt, path, measured):
    fig, axes = plt.subplots(1, 2, figsize=(11.7, 4.5), layout="constrained")
    for axis, field, label in ((axes[0], "score_calls", "New fixed grid score calls"),
                               (axes[1], "grid_s", "Measured grid wall time (s)")):
        values = [[float(row[field]) for row in measured if row["method"] == method] for method in METHODS]
        axes_with_data = [(method, value) for method, value in zip(METHODS, values) if value]
        if axes_with_data:
            axis.boxplot([v for _, v in axes_with_data], labels=[m for m, _ in axes_with_data], showmeans=True)
        axis.set_ylabel(label)
        if field == "score_calls":
            axis.axhline(4851, c="#ca691f", ls="--", lw=1, label="4,851 per frame")
            axis.legend(fontsize=8)
    fig.suptitle("Q2 measured resource use | S0 fixed grid is reused")
    fig.savefig(path / "runtime_and_score_budget.png", dpi=145)
    plt.close(fig)


def figure_raw309_scale(plt, path, records):
    if not records:
        return
    fig, axes = plt.subplots(1, 2, figsize=(11.8, 4.9), layout="constrained")
    aliases = {"S0": "C0", "S1": "C3", "S2": "C3", "S3": "C3"}
    for axis, metric in zip(axes, METRICS):
        for method in METHODS:
            selected = [
                row for row in records if row["method"] == method and row["metric"] == metric
                and row["policy"] == aliases[method] and row["stage"] == "final_winner"
            ]
            if not selected:
                continue
            axis.plot([row["range_m"] for row in selected],
                      [row["translation_plus_rotation_cm"] for row in selected],
                      marker="o", label=f"{method}/{aliases[method]}")
        axis.axhline(5.0, color="#8c5831", ls="--", label="5 cm comparison scale")
        axis.set(xlabel="Range (m)", ylabel="Translation + first-order angular displacement (cm)",
                 title=f"{metric.title()} final selected pose", yscale="log")
        axis.legend(fontsize=8)
    fig.suptitle("raw309 | error propagation from actual offline pose discrepancies")
    fig.savefig(path / "raw309_microstructure_error_scales.png", dpi=145)
    plt.close(fig)


def figure_runtime_components(plt, path, measured, fits):
    refit_scoring = Counter()
    for fit in fits:
        if fit["origin"] == "Q2_NEW":
            refit_scoring[(fit["sample_id"], fit["method"])] += float(fit["postfit_score_s"])
    fig, axes = plt.subplots(1, 2, figsize=(12.4, 4.8), layout="constrained")
    labels = (
        ("transform_s", "Transform", "#155b93"),
        ("query_s", "Voxel queries (shared interval)", "#1e887e"),
        ("native_s", "Native kernel", "#4fac79"),
        ("park_s", "Park kernel", "#727abb"),
        ("extract_s", "Final peak extraction", "#d88123"),
        ("fit_s", "Native align", "#aa5270"),
        ("postfit_score_s", "Terminal rescore", "#8c5441"),
        ("io_s", "CSV I/O", "#727881"),
    )
    grouped = {method: [row for row in measured if row["method"] == method] for method in METHODS}
    for axis, methods in zip(axes, (METHODS, ("S0",))):
        positions = np.arange(len(methods))
        bottom = np.zeros(len(methods))
        for field, name, color in labels:
            values = []
            for method in methods:
                observations = grouped[method]
                metric = (
                    [refit_scoring[(row["sample_id"], method)] for row in observations]
                    if field == "postfit_score_s" else [float(row[field]) for row in observations]
                )
                values.append(float(np.mean(metric)) if metric else 0.0)
            axis.bar(positions, values, bottom=bottom, label=name, width=.62, color=color)
            bottom += np.asarray(values)
        axis.set(xticks=positions, xticklabels=methods, ylabel="Mean measured component time per frame (s)")
    axes[0].set_title("All methods")
    axes[1].set_title("S0 incremental work · expanded scale")
    axes[0].legend(fontsize=7, ncols=2)
    fig.suptitle("Compute decomposition | shared map/scan preprocessing reported separately")
    fig.savefig(path / "runtime_component_breakdown.png", dpi=150)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--online", required=True, type=Path)
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--grid", required=True, type=Path)
    parser.add_argument("--retained", required=True, type=Path)
    parser.add_argument("--fits", required=True, type=Path)
    parser.add_argument("--runtime", required=True, type=Path)
    parser.add_argument("--reference", required=True, type=Path)
    parser.add_argument("--seal", required=True, type=Path)
    parser.add_argument("--q1-run", required=True, type=Path)
    parser.add_argument("--out", required=True, type=Path)
    args = parser.parse_args()

    validate_seal(args)
    frames = read_inputs(args.online)
    manifest = {row["sample_id"]: row for row in load_csv(args.manifest)}
    if set(frames) != set(manifest) or len(frames) != 12:
        raise ValueError("sample inventory is not the frozen twelve")
    verified_assets = {}
    for sample, frame in frames.items():
        for asset, field in ((frame.map_path, "map_sha256"), (frame.scan_path, "prepared_pcd_sha256")):
            if asset not in verified_assets:
                verified_assets[asset] = digest(asset)
            if verified_assets[asset] != manifest[sample][field]:
                raise ValueError(f"{sample}: input changed from Q1 manifest: {asset}")
    grids = read_grid_sources(args.grid, args.q1_run)
    modes = read_retention(args.retained, args.q1_run)
    if set(grids) != {(sample, method) for sample in frames for method in METHODS}:
        raise ValueError("the frame-by-method grid inventory is incomplete")
    fits = read_fits(args.fits, args.q1_run, grids)
    if set(fits) - set(grids):
        raise ValueError("fit method/frame does not belong to the frozen grid inventory")
    references = read_reference(args.reference)
    if set(references) - set(frames):
        raise ValueError("reference ID outside frozen sample inventory")
    measured, runtime = summarize_runtime(args.runtime)
    if {(row["sample_id"], row["method"]) for row in measured} != set(grids) or len(measured) != len(grids):
        raise ValueError("timing inventory is incomplete or duplicated")
    for row in measured:
        key = row["sample_id"], row["method"]
        if int(row["unique_poses"]) != len(grids[key]):
            raise ValueError(f"{key}: recorded scored grid count mismatch")
        expected_fresh_calls = 0 if row["method"] == "S0" else len(grids[key])
        if any(int(row[field]) != expected_fresh_calls for field in ("score_calls", "native_calls", "park_calls", "transform_calls")):
            raise ValueError(f"{key}: fixed score call inventory differs")
        if int(row["align_calls"]) != sum(fit["origin"] == "Q2_NEW" for fit in fits[key]):
            raise ValueError(f"{key}: recorded alignment count mismatch")

    coverage, ranks, refits, evaluated_grid = evaluate_matrix(grids, modes, fits, references, manifest)
    del grids
    primary = [row for row in coverage if (row["translation_tol_m"], row["rotation_tol_deg"]) == PRIMARY]
    grouped = aggregate(coverage)
    paired = compare_retention(primary)
    raw309 = record_raw309(evaluated_grid, frames, references, primary)
    microstructure = microstructure_records(primary, refits)
    table = [
        row for row in grouped
        if row["group"] in ("ALL", "B") and (row["translation_tol_m"], row["rotation_tol_deg"]) == PRIMARY
        and row["policy"] == ("C0" if row["method"] == "S0" else "C3")
    ]
    fit_counts = [
        {
            "method": method,
            "new_trials": sum(f["method"] == method and f["origin"] == "Q2_NEW" for f in refits),
            "qualified_native_trials": sum(f["method"] == method and f["qualified_native_fit"] for f in refits),
            "mode_preserved_native_accepted": sum(f["method"] == method and f["accepted_native"] for f in refits),
            "mode_preserved_park_accepted": sum(f["method"] == method and f["accepted_park"] for f in refits),
            "native_accepted_correct": sum(f["method"] == method and f["accepted_native"] and good(f) for f in refits),
        }
        for method in METHODS
    ]
    summary = {
        "reference_input_sha256": digest(args.reference),
        "sealed_input_manifest_sha256": digest(args.seal),
        "sample_count": len(manifest), "valid_reference_count": len(references),
        "primary_main_comparison": table,
        "raw309": raw309,
        "runtime": runtime,
        "fit_counts": fit_counts,
    }

    args.out.mkdir(parents=True, exist_ok=False)
    write_csv(args.out / "candidate_recall.csv", coverage)
    write_csv(args.out / "candidate_recall_summary.csv", grouped)
    write_csv(args.out / "retention_policy_comparison.csv", paired)
    write_csv(args.out / "final_ranking_comparison.csv", ranks)
    write_csv(args.out / "refinement_offline_errors.csv", refits)
    write_csv(args.out / "runtime_summary.csv", runtime)
    if raw309:
        write_csv(args.out / "raw309_center_and_coverage.csv", raw309)
    if microstructure:
        write_csv(args.out / "raw309_microstructure_scale.csv", microstructure)
    (args.out / "evaluation_summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    figures = args.out / "figures"
    figures.mkdir()
    figure_coverage(plt, figures, primary, manifest)
    figure_retention(plt, figures, primary)
    figure_raw309(plt, figures, evaluated_grid, frames, references)
    figure_refinement(plt, figures, refits, evaluated_grid)
    figure_runtime(plt, figures, measured)
    figure_runtime_components(plt, figures, measured, refits)
    figure_raw309_scale(plt, figures, microstructure)


if __name__ == "__main__":
    main()
