"""Select and score shared source masks without reference labels, then seal output."""

import argparse
import csv
import hashlib
import json
import math
import time
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from freeze import PAIRS

BUDGETS = (20, 50, 80)
RANDOM_SEEDS = tuple(range(6400, 6410))
RANK_EPS = 1e-12
INFO_THRESHOLD = 1e-6
GEOMETRY = {"GEO0": 0, "GEO1": 1, "B8": 2, "B9": 3}


@dataclass
class Selection:
    method: str
    budget_percent: int
    target_n: int
    seed: int
    indices: np.ndarray
    support_n: int
    fallback_n: int = 0


def sha256(path):
    with path.open("rb") as stream:
        hashed = hashlib.sha256()
        for block in iter(lambda: stream.read(1 << 20), b""):
            hashed.update(block)
        return hashed.hexdigest()


def write_csv(path, rows):
    fields = list(dict.fromkeys(key for row in rows for key in row))
    if not fields:
        raise ValueError(f"empty CSV: {path}")
    with path.open("x", newline="") as stream:
        writer = csv.DictWriter(stream, fields, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def salience_order(values, eligible=None):
    indices = np.arange(len(values)) if eligible is None else np.flatnonzero(eligible)
    if not np.isfinite(values[indices]).all():
        raise ValueError("non-finite eligible point salience")
    return indices[np.lexsort((indices, -values[indices]))]


def voxel_interleave(source, width):
    by_cell = defaultdict(list)
    for point, voxel in enumerate(np.floor(source / width).astype(np.int64)):
        by_cell[tuple(voxel)].append(point)
    cells = [by_cell[key] for key in sorted(by_cell)]
    sequence = []
    for slot in range(max(map(len, cells))):
        sequence.extend(cell[slot] for cell in cells if slot < len(cell))
    return np.asarray(sequence, dtype=np.int64)


def fill_budget(priority, fallback, amount):
    used = np.zeros(len(fallback), dtype=bool)
    used[priority] = True
    selected = np.concatenate([priority[:amount], fallback[~used[fallback]][:max(0, amount - len(priority))]])
    if len(selected) != amount or len(np.unique(selected)) != amount:
        raise ValueError("invalid matched point budget")
    return selected


def score_explanations(score_left, score_right):
    left, right = score_left[:13], score_right[:13]
    delta = left - right
    average = delta.mean(axis=0)
    low, high = np.quantile(delta, (0.2, 0.8), axis=0)
    return {
        "E0": np.abs(delta[0]),
        "E1": np.abs(average),
        "E2": np.abs(average) / np.sqrt(left.var(axis=0, ddof=1) + right.var(axis=0, ddof=1) + 0.05 ** 2),
        "E3": np.maximum.reduce([np.zeros_like(average), low, -high]),
        "mean_signed": average,
        "paired_sign_stable": (low > 0) | (high < 0),
    }


def explanation_entropy(states):
    both_known = (states != 0).all(axis=0)
    # For two candidates, a differing two-state explanation yields p=0.5.
    entropy = np.where(both_known & (states[0] != states[1]), 1.0, 0.0)
    return entropy, both_known


def information_leverage(per_point_info):
    outputs = []
    for candidate in per_point_info:
        full = candidate.sum(axis=0)
        ridge = max(np.linalg.eigvalsh(full)[-1] * INFO_THRESHOLD, 1e-12)
        whitened = np.linalg.inv(full + ridge * np.eye(6))
        outputs.append(np.maximum(0.0, np.einsum("ij,nji->n", whitened, candidate)))
    return np.mean(outputs, axis=0)


def derive_selections(source, score, states, information, geometry):
    size = len(source)
    order = voxel_interleave(source, 0.30)
    attributes = score_explanations(score[0], score[1])
    entropy, known = explanation_entropy(states)
    attributes.update(B5_entropy=entropy, B5_eligible=known, B6_leverage=information_leverage(information))
    neutral = attributes["E0"] <= 0.05
    natural = np.flatnonzero(~neutral)
    yield Selection("B0", 100, size, -1, np.arange(size), size), attributes
    yield Selection("B3_NATURAL", -1, len(natural), -1, natural, len(natural)), attributes
    priors = {
        "B3_EQUAL": salience_order(attributes["E0"]),
        "B4": salience_order(attributes["E0"]),
        "B5": salience_order(entropy, known),
        "B6": salience_order(attributes["B6_leverage"]),
        **{f"B7_{key}": salience_order(attributes[key]) for key in ("E1", "E2", "E3")},
    }
    for name, slot in GEOMETRY.items():
        geom_score = geometry["scores"][:, slot]
        supported = geometry["observable"][:, slot] & (geometry["classes"][:, slot] == 2)
        priors[name] = salience_order(geom_score, supported)
        attributes[f"{name}_geometry_supported"] = supported
        attributes[f"{name}_geometry_salience"] = geom_score

    for percent in BUDGETS:
        amount = math.ceil(percent * size / 100)
        for seed in RANDOM_SEEDS:
            chosen = np.random.default_rng(seed).permutation(size)[:amount]
            yield Selection("B1", percent, amount, seed, chosen, amount), attributes
        yield Selection("B2", percent, amount, -1, order[:amount], amount), attributes
        for name, priority in priors.items():
            selected = fill_budget(priority, order, amount)
            support_n = min(len(priority), amount)
            yield Selection(name, percent, amount, -1, selected, support_n, amount - support_n), attributes
            if name in GEOMETRY:
                yield Selection(name + "_SUPPORTED_ONLY", percent, amount, -1, priority[:amount],
                                support_n), attributes


def input_inventory(run):
    paths = {}
    for frame in PAIRS:
        paths[frame] = {
            "score": run / f"{frame}.npz",
            "observation": run / f"observation_{frame}.npz",
            "geometry": run / "geometry" / f"geometry_{frame}.npz",
        }
    return paths


def read_frame(files, manifests):
    with np.load(files["score"], allow_pickle=False) as archive:
        data = {name: archive[name] for name in ("source", "score", "associated", "info", "poses", "nominal")}
    with np.load(files["observation"], allow_pickle=False) as receipt:
        state = receipt["state"]
        confidence = receipt["confidence"]
    with np.load(files["geometry"], allow_pickle=False) as receipt:
        geometry = {name: receipt[name] for name in ("pairs", "pair_scores", "pair_classes", "pair_observable")}
    n = len(data["source"])
    k = len(manifests)
    if (
        n == 0 or data["source"].shape != (n, 3) or data["score"].shape != (k, 26, n)
        or data["associated"].shape != (k, 26, n) or data["info"].shape != (k, n, 6, 6)
        or data["poses"].shape != (k, 26, 4, 4) or data["nominal"].shape != (k, 4, 4)
        or state.shape != (k, n) or confidence.shape != (k, n)
        or geometry["pair_scores"].shape[1:] != (n, 4)
        or geometry["pair_classes"].shape != geometry["pair_scores"].shape
        or geometry["pair_observable"].shape != geometry["pair_scores"].shape
    ):
        raise ValueError(f"incompatible frozen frame arrays: {files['score']}")
    expected = np.asarray([[float(row[f"T{i}{j}"]) for i in range(4) for j in range(4)] for row in manifests])
    if not np.array_equal(expected.astype(data["nominal"].dtype).reshape(k, 4, 4), data["nominal"]):
        raise ValueError("score arrays have different hypotheses or order from frozen manifest")
    if not np.array_equal(data["score"][:, 0], data["score"][:, 13]):
        raise ValueError("nominal scorer identity differs between perturbation scales")
    if not np.isin(state, [0, 1, 2]).all() or not np.isin(geometry["pair_classes"], [0, 1, 2]).all():
        raise ValueError("invalid explanation or geometry state codes")
    if not all(np.isfinite(data[key]).all() for key in ("source", "score", "info")):
        raise ValueError("non-finite source, scores or information")
    if np.any(data["score"][~data["associated"].astype(bool)] != 0):
        raise ValueError("native no-association reward changed from zero")
    data.update(observation=state, confidence=confidence, geometry=geometry)
    return data


def geometry_for_pair(data, left, right):
    geom = data["geometry"]
    slots = np.flatnonzero((geom["pairs"] == sorted((left, right))).all(axis=1))
    if len(slots) != 1:
        raise ValueError(f"expected a unique geometry record for pair {left}, {right}")
    slot = int(slots[0])
    return {
        "scores": geom["pair_scores"][slot],
        "classes": geom["pair_classes"][slot],
        "observable": geom["pair_observable"][slot].astype(bool),
    }


def sign_summary(delta):
    positive = float(delta[delta > 0].sum())
    negative = float(delta[delta < 0].sum())
    return {
        "positive_points": int((delta > RANK_EPS).sum()),
        "negative_points": int((delta < -RANK_EPS).sum()),
        "equal_points": int((np.abs(delta) <= RANK_EPS).sum()),
        "gross_abs": positive - negative,
        "sum_pos": positive,
        "sum_neg": negative,
        "net": float(delta.sum()),
    }


def result_sign(values):
    return np.where(values > RANK_EPS, 1, np.where(values < -RANK_EPS, -1, 0))


def point_segments(source, width):
    coordinates = np.floor(source / width).astype(np.int64)
    _, codes = np.unique(coordinates, axis=0, return_inverse=True)
    return codes


def information_rows(context, info, full_eigenvalues, selection, ids):
    records = []
    chosen = selection.indices
    for index, candidate in enumerate(ids):
        scale = max(full_eigenvalues[index, -1] * INFO_THRESHOLD, 1e-12)
        full_min = full_eigenvalues[index, 0]
        matrix = info[index, chosen].sum(axis=0)
        eigen, vectors = np.linalg.eigh((matrix + matrix.T) / 2)
        direction = vectors[:, 0].copy()
        largest = np.argmax(np.abs(direction))
        if direction[largest] < 0:
            direction *= -1
        # Repeated weakest eigenvalues do not identify a unique eigenvector.
        unique = (eigen[1] - eigen[0]) > scale
        weakest_ratio = float(eigen[0] / full_min) if full_min > scale else None
        equal_ratio = weakest_ratio * info.shape[1] / len(chosen) if weakest_ratio is not None and len(chosen) else None
        records.append({
            **context, "candidate_id": candidate, "rank": int((eigen > scale).sum()),
            "rank_abs_threshold": float(scale), "lambda_min_ratio": weakest_ratio,
            "lambda_min_ratio_at_equal_count": equal_ratio,
            "trace_ratio": float(eigen.sum() / full_eigenvalues[index].sum()) if full_eigenvalues[index].sum() > scale else None,
            "translation_rotation_coupling_fro": float(np.linalg.norm(matrix[:3, 3:])),
            "condition_number": float(eigen[-1] / eigen[0]) if eigen[0] > scale else None,
            "weakest_direction_unique": unique,
            "weakest_mode_translation_norm": float(np.linalg.norm(direction[:3])) if unique else None,
            "weakest_mode_scaled_rotation_norm": float(np.linalg.norm(direction[3:])) if unique else None,
            **{f"lambda_{d + 1}": float(value) for d, value in enumerate(eigen)},
            **{f"weakest_component_{d + 1}": float(component) if unique else None for d, component in enumerate(direction)},
        })
    return records


def pair_row(context, selection, data, left, right, space_codes, plane_codes, geometry):
    chosen = np.sort(selection.indices)
    n = len(chosen)
    nominal = data["score"][:, 0, chosen].sum(axis=-1)
    scores = data["score"][:, 13:26, chosen].sum(axis=-1)
    delta = data["score"][left, 0, chosen] - data["score"][right, 0, chosen]
    summary = sign_summary(delta)
    difference = scores[left] - scores[right]
    matrix_difference = scores[left, :, None] - scores[right, None, :]
    paired_sign = result_sign(difference)
    cartesian = result_sign(matrix_difference)
    support = data["associated"][:, 0, chosen].astype(bool)
    nominal_state = data["observation"][:, chosen]
    row = {
        **context, "selected_n": n, "target_n": selection.target_n,
        "fraction_of_full": float(n / data["source"].shape[0]),
        "priority_supported_n": selection.support_n, "fallback_n": selection.fallback_n,
        "score_a": float(nominal[left]), "score_b": float(nominal[right]),
        "pair_margin_per_retained_point": summary["net"] / n if n else None,
        "pair_preferred": int(result_sign(summary["net"])) if n else None,
        "left_native_associated": int(support[left].sum()), "right_native_associated": int(support[right].sum()),
        "both_native_associated": int((support[left] & support[right]).sum()),
        "both_explanation_known": int(((nominal_state[left] > 0) & (nominal_state[right] > 0)).sum()),
        "occupied_voxels_030": len(np.unique(space_codes[chosen])),
        "occupied_patches_050": len(np.unique(plane_codes[chosen])),
        "geo2_observable": int(geometry["observable"][chosen, 2].sum()),
        "geo2_common": int((geometry["classes"][chosen, 2] == 1).sum()),
        "geo3_observable": int(geometry["observable"][chosen, 3].sum()),
        "geo3_common": int((geometry["classes"][chosen, 3] == 1).sum()),
        "geo2_discriminative": int((geometry["classes"][chosen, 2] == 2).sum()),
        "geo3_discriminative": int((geometry["classes"][chosen, 3] == 2).sum()),
        "scale2_paired_a_wins": int((paired_sign[1:] > 0).sum()) if n else None,
        "scale2_paired_b_wins": int((paired_sign[1:] < 0).sum()) if n else None,
        "scale2_paired_ties": int((paired_sign[1:] == 0).sum()) if n else None,
        "scale2_paired_delta_min": float(difference[1:].min()) if n else None,
        "scale2_paired_delta_max": float(difference[1:].max()) if n else None,
        "scale2_cross_a_wins": int((cartesian[1:, 1:] > 0).sum()) if n else None,
        "scale2_cross_b_wins": int((cartesian[1:, 1:] < 0).sum()) if n else None,
        "scale2_cross_ties": int((cartesian[1:, 1:] == 0).sum()) if n else None,
        **summary,
    }
    if not np.isclose(row["score_a"] - row["score_b"], summary["net"], rtol=1e-11, atol=1e-9):
        raise ValueError("same-mask paired raw totals and contributions differ")
    return row, nominal, scores


def run_blind(args):
    paths = input_inventory(args.run)
    frame_hypotheses = defaultdict(list)
    with args.hypotheses.open(newline="") as stream:
        for row in csv.DictReader(stream):
            frame_hypotheses[row["frame"]].append(row)
    if set(frame_hypotheses) != set(PAIRS):
        raise ValueError("hypothesis manifest does not contain all frozen frames")
    args.out.mkdir(parents=True, exist_ok=False)
    orders, score_records, info_records = [], [], []
    masks, attributes_store = {}, {}
    timings = []
    for frame, files in paths.items():
        entries = frame_hypotheses[frame]
        ids = [row["candidate_id"] for row in entries]
        if len(ids) != len(set(ids)):
            raise ValueError(f"duplicate hypothesis ID: {frame}")
        data = read_frame(files, entries)
        space = point_segments(data["source"], 0.30)
        planes = point_segments(data["source"], 0.50)
        hessians = data["info"].sum(axis=1)
        all_eigen = np.linalg.eigvalsh((hessians + hessians.transpose(0, 2, 1)) / 2)
        for pair, left, right in PAIRS[frame]:
            geom = geometry_for_pair(data, left, right)
            tick = time.perf_counter()
            options = list(derive_selections(
                data["source"], data["score"][[left, right]], data["observation"][[left, right]],
                data["info"][[left, right]], geom,
            ))
            select_s = time.perf_counter() - tick
            tick = time.perf_counter()
            for number, (selection, attributes) in enumerate(options):
                tag = f"{frame}_{pair}_{number:03d}"
                context = {
                    "frame": frame, "pair": pair, "candidate_a": ids[left], "candidate_b": ids[right],
                    "method": selection.method, "budget_percent": selection.budget_percent,
                    "seed": selection.seed, "mask_id": tag,
                }
                masks[tag] = np.sort(selection.indices)
                if number == 1:
                    for key, values in attributes.items():
                        attributes_store[f"{frame}_{pair}_{key}"] = values
                row, nominal, scores = pair_row(context, selection, data, left, right, space, planes, geom)
                row["scale1_sign_stable_fraction"] = (float(attributes["paired_sign_stable"][selection.indices].mean())
                                                      if len(selection.indices) else None)
                orders.append(row)
                for k, candidate in enumerate(ids):
                    score_records.append({
                        **context, "candidate_id": candidate,
                        "nominal_score": float(nominal[k]),
                        **{f"scale2_score_{j:02d}": float(value) for j, value in enumerate(scores[k])},
                    })
                info_records.extend(information_rows(context, data["info"], all_eigen, selection, ids))

            timings.append(dict(frame=frame, pair=pair, mask_count=len(options), selection_s=select_s,
                                cached_score_and_info_s=time.perf_counter()-tick))

    write_csv(args.out / "blind_runtime.csv", timings)
    write_csv(args.out / "blind_pair_ranking.csv", orders)
    write_csv(args.out / "blind_candidate_scores.csv", score_records)
    write_csv(args.out / "blind_information.csv", info_records)
    np.savez_compressed(args.out / "shared_masks.npz", **masks)
    np.savez_compressed(args.out / "blind_point_evidence.npz", **attributes_store)
    seal_paths = tuple(args.out.iterdir()) + (
        args.hypotheses, Path(__file__).resolve(), Path(__file__).with_name("freeze.py"),
        *(p for values in paths.values() for p in values.values()),
    )
    receipt = {
        "notice": "Reference labels were not opened and were not available to the selection function.",
        "files": {str(path.resolve()): sha256(path) for path in seal_paths},
        "mask_count": len(masks),
        "score_rows": len(score_records),
        "pair_order_rows": len(orders),
        "candidate_information_rows": len(info_records),
    }
    with (args.out / "blind_seal.json").open("x") as stream:
        json.dump(receipt, stream, indent=2)
        stream.write("\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--hypotheses", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    run_blind(parser.parse_args())


if __name__ == "__main__":
    main()
