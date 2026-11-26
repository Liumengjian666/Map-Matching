"""Find supported, deterministic grid peaks before any reference data is read."""

import argparse
import csv
import heapq
import math
from collections import defaultdict
from pathlib import Path

import numpy as np


SHAPE = (21, 21, 11)
STEPS = (0.2, 0.2, 1.0)
POSE_FIELDS = tuple(f"T{r}{c}" for r in range(4) for c in range(4))
METRICS = ("native", "park")
MODE_FIELDS = (
    "sample_id", "metric", "index", "rank", "retained", "mode_class",
    "filter_reason", "local_score", "relative_score", "support", "boundary",
    "plateau_size", "saddle_to_best", "saddle_relative_to_peak",
    "independent_competitor_count", "second_index", "second_separation_m",
    "second_separation_deg", "second_saddle", "curvature_x", "curvature_y",
    "curvature_yaw",
)


def grid_coordinates(row):
    return tuple(int(row[field]) for field in ("ix", "iy", "ia"))


def grid_offset(row):
    return tuple(float(row[field]) for field in ("dx", "dy", "dyaw"))


def read_grid(path):
    grouped = defaultdict(list)
    with path.open(newline="") as stream:
        for row in csv.DictReader(stream):
            grouped[row["sample_id"]].append(row)
    if not grouped:
        raise ValueError("grid is empty")
    for sample, rows in grouped.items():
        indices = [int(row["index"]) for row in rows]
        if len(rows) != math.prod(SHAPE) or len(set(indices)) != len(rows):
            raise ValueError(f"{sample}: invalid grid index inventory")
        for axis in range(3):
            lower = min(grid_coordinates(row)[axis] for row in rows)
            for row in rows:
                row[f"_axis{axis}"] = grid_coordinates(row)[axis] - lower
        coords = [tuple(row[f"_axis{i}"] for i in range(3)) for row in rows]
        if len(set(coords)) != len(rows) or any(
            not all(0 <= x < bound for x, bound in zip(coord, SHAPE)) for coord in coords
        ):
            raise ValueError(f"{sample}: invalid grid coordinates")
        for row, coord in zip(rows, coords):
            expected = tuple((i - dim // 2) * step for i, dim, step in zip(coord, SHAPE, STEPS))
            if not np.allclose(grid_offset(row), expected, rtol=0, atol=1e-7):
                raise ValueError(f"{sample}: unexpected grid offsets at {coord}")
    return grouped


def node_index(row):
    return tuple(row[f"_axis{i}"] for i in range(3))


def neighbors(coord):
    for axis, bound in enumerate(SHAPE):
        for step in (-1, 1):
            position = coord[axis] + step
            if 0 <= position < bound:
                yield coord[:axis] + (position,) + coord[axis + 1:]


def is_boundary(coord):
    return any(value in (0, bound - 1) for value, bound in zip(coord, SHAPE))


def support(row, metric="native"):
    count = int(row["total"])
    field = "park_associated" if metric == "park" and row.get("park_associated") else "associated"
    associated = int(row[field])
    if count <= 0 or not 0 <= associated <= count:
        raise ValueError(f"invalid support at {row['sample_id']} index {row['index']}")
    return associated / count


def plateau_peaks(values):
    visited = set()
    for origin in sorted(values):
        if origin in visited:
            continue
        level = values[origin]
        plateau = {origin}
        visited.add(origin)
        pending = [origin]
        higher = False
        while pending:
            node = pending.pop()
            for neighbor in neighbors(node):
                if neighbor not in values:
                    continue
                value = values[neighbor]
                higher |= value > level
                if value == level and neighbor not in visited:
                    visited.add(neighbor)
                    plateau.add(neighbor)
                    pending.append(neighbor)
        if not higher:
            # An interior point of a plateau touching the boundary still lacks a
            # closed trust region, so preserve boundary provenance separately.
            center = tuple(dim // 2 for dim in SHAPE)
            representative = min(
                plateau,
                key=lambda p: (
                    is_boundary(p),
                    sum(((x - m) * s) ** 2 for x, m, s in zip(p, center, (0.2, 0.2, 0.1))),
                    p,
                ),
            )
            yield representative, len(plateau), any(map(is_boundary, plateau))


def widest_paths(values, origin):
    """Best achievable bottleneck score along a supported 6-neighbor path."""
    capacity = {origin: values[origin]}
    queue = [(-values[origin], origin)]
    while queue:
        negative, node = heapq.heappop(queue)
        current = -negative
        if current < capacity[node]:
            continue
        for other in neighbors(node):
            if other not in values:
                continue
            chance = min(current, values[other])
            if other not in capacity or chance > capacity[other]:
                capacity[other] = chance
                heapq.heappush(queue, (-chance, other))
    return capacity


def separation(first, second):
    x1, y1, a1 = grid_offset(first)
    x2, y2, a2 = grid_offset(second)
    return math.hypot(x2 - x1, y2 - y1), abs(a2 - a1)


def near_peak_connected_span(near, start):
    component = {start}
    pending = [start]
    while pending:
        for neighbor in neighbors(pending.pop()):
            if neighbor in near and neighbor not in component:
                component.add(neighbor)
                pending.append(neighbor)
    axes = np.array(tuple(component))
    ranges = np.ptp(axes, axis=0) * STEPS
    return len(component), max(ranges[:2]), ranges[2]


def curvature(coord, values, axis):
    i = coord[axis]
    if i == 0 or i == SHAPE[axis] - 1:
        return ""
    lower = coord[:axis] + (i - 1,) + coord[axis + 1:]
    higher = coord[:axis] + (i + 1,) + coord[axis + 1:]
    if lower not in values or higher not in values:
        return ""
    return (values[higher] + values[lower] - 2 * values[coord]) / STEPS[axis] ** 2


def extract_metric(rows, metric):
    by_coord = {node_index(row): row for row in rows}
    original = {
        coord: float(row[metric])
        for coord, row in by_coord.items()
        if support(row, metric) >= 0.5 and math.isfinite(float(row[metric]))
    }
    if not original or (metric == "native" and max(original.values()) <= 0):
        return [{"sample_id": rows[0]["sample_id"], "metric": metric, "mode_class": "NO_VALID_MODE"}]

    best_score = max(original.values())
    relative = {
        coord: (value / best_score if metric == "native" else math.exp(0.001 * (value - best_score)))
        for coord, value in original.items()
    }
    peaks = sorted(plateau_peaks(original), key=lambda peak: (-original[peak[0]], peak[0]))
    records = []
    retained = []
    best_coord = peaks[0][0]
    capacities = widest_paths(relative, best_coord)

    for coord, size, boundary in peaks:
        row = by_coord[coord]
        retained_rank = ""
        reason = "retained"
        if any(
            distance <= 0.30 + 1e-12 and angle <= 3.0
            for distance, angle in (separation(row, by_coord[other]) for other in retained)
        ):
            reason = "nonmax"
        elif len(retained) == 5:
            reason = "top5_budget"
        else:
            retained.append(coord)
            retained_rank = len(retained)

        saddle = capacities.get(coord)
        records.append(dict(
            sample_id=row["sample_id"], metric=metric, index=row["index"],
            rank=retained_rank, retained=bool(retained_rank), filter_reason=reason,
            local_score=original[coord], relative_score=relative[coord], support=support(row, metric),
            boundary=boundary, plateau_size=size, saddle_to_best="" if saddle is None else saddle,
            saddle_relative_to_peak="" if saddle is None or relative[coord] <= 0 else saddle / relative[coord],
            curvature_x=curvature(coord, original, 0), curvature_y=curvature(coord, original, 1),
            curvature_yaw=curvature(coord, original, 2),
        ))

    # A near-score band and saddle test describe competition around the best
    # peak. Disconnected support contributes uncertainty, not a measured saddle.
    near = {coord for coord, value in relative.items() if value >= 0.95 - 1e-12}
    count, span_m, span_deg = near_peak_connected_span(near, best_coord)
    competitors = [
        coord for coord in retained[1:]
        if coord in near and coord in capacities and capacities[coord] <= 0.95 * relative[coord] + 1e-12
    ]
    boundary_by_coord = {coord: boundary for coord, _, boundary in peaks}
    interior_competitors = [coord for coord in competitors if not boundary_by_coord[coord]]
    index_of_second = ""
    distance = angle = other_saddle = ""
    if len(retained) >= 2:
        runner = retained[1]
        index_of_second = by_coord[runner]["index"]
        distance, angle = separation(by_coord[best_coord], by_coord[runner])
        other_saddle = capacities.get(runner, "")

    if count > len(near) / 2 and (span_m >= 0.8 - 1e-12 or span_deg >= 4.0):
        mode_class = "FLAT"
    elif boundary_by_coord[best_coord]:
        mode_class = "UNKNOWN"
    elif interior_competitors:
        mode_class = "MULTI_MODE"
    elif any(coord in near and coord not in capacities for coord in retained[1:]) or competitors:
        mode_class = "UNKNOWN"
    else:
        mode_class = "SINGLE_MODE"
    for record in records:
        record.update(
            mode_class=mode_class,
            independent_competitor_count=len(interior_competitors),
            second_index=index_of_second,
            second_separation_m=distance,
            second_separation_deg=angle,
            second_saddle=other_saddle,
        )
    return records


def refinement_seeds(rows, by_metric):
    by_index = {str(row["index"]): row for row in rows}
    entries = [
        (int(record["rank"]), metric_order, record, metric)
        for metric_order, metric in enumerate(METRICS)
        for record in by_metric[metric]
        if record.get("retained") and not record["boundary"]
    ]
    entries.sort(key=lambda entry: entry[:2])
    if not entries:
        # Boundary or unsupported starts can diagnose a failure but do not
        # furnish independent interior-mode evidence.
        finite = [row for row in rows if math.isfinite(float(row["native"]))]
        if finite:
            best = min(finite, key=lambda row: (-float(row["native"]), int(row["index"])))
            return [(best, 1, "native_diagnostic")]
        return []
    chosen = {}
    for rank, _, record, metric in entries:
        key = str(record["index"])
        if key not in chosen:
            chosen[key] = (by_index[key], rank, metric)
        if len(chosen) == 4:
            break
    return list(chosen.values())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("grid", type=Path)
    parser.add_argument("modes", type=Path)
    parser.add_argument("fit_seeds", type=Path)
    args = parser.parse_args()
    grouped = read_grid(args.grid)
    with args.modes.open("x", newline="") as modes, args.fit_seeds.open("x") as output:
        writer = csv.DictWriter(modes, MODE_FIELDS)
        writer.writeheader()
        for sample, rows in grouped.items():
            by_metric = {metric: extract_metric(rows, metric) for metric in METRICS}
            for records in by_metric.values():
                writer.writerows(records)
            for row, rank, metric in refinement_seeds(rows, by_metric):
                fields = [sample, row["index"], str(rank), metric] + [row[field] for field in POSE_FIELDS]
                output.write("\t".join(fields) + "\n")


if __name__ == "__main__":
    main()
