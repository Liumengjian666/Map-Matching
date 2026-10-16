#!/usr/bin/env python3
"""Render fixed A/B P43 frame overlays; performs no registration or scoring."""
import argparse
import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def load_binary_xyz(path):
    path = Path(path)
    fields = {}
    with path.open("rb") as stream:
        while True:
            line = stream.readline()
            if not line:
                raise ValueError(f"incomplete PCD header: {path}")
            text = line.decode("ascii").strip()
            if text.startswith("FIELDS "):
                fields["names"] = text.split()[1:]
            elif text.startswith("SIZE "):
                fields["sizes"] = [int(value) for value in text.split()[1:]]
            elif text.startswith("TYPE "):
                fields["types"] = text.split()[1:]
            elif text.startswith("COUNT "):
                fields["counts"] = [int(value) for value in text.split()[1:]]
            elif text.startswith("POINTS "):
                fields["points"] = int(text.split()[1])
            elif text.startswith("DATA "):
                if text.split()[1] != "binary":
                    raise ValueError(f"expected binary PCD: {path}")
                fields["data_offset"] = stream.tell()
                break
    names = fields["names"]
    sizes = fields["sizes"]
    types = fields["types"]
    counts = fields.get("counts", [1] * len(names))
    offsets = []
    point_step = 0
    dtype_map = {("F", 4): "<f4", ("F", 8): "<f8",
                 ("U", 1): "u1", ("U", 2): "<u2", ("U", 4): "<u4",
                 ("I", 1): "i1", ("I", 2): "<i2", ("I", 4): "<i4"}
    dtype_fields = []
    for name, size, kind, count in zip(names, sizes, types, counts):
        offsets.append(point_step)
        if (kind, size) not in dtype_map:
            raise ValueError(f"unsupported PCD field {name}: {kind}{size}")
        dtype_fields.append((name, dtype_map[(kind, size)], (count,) if count > 1 else ()))
        point_step += size * count
    dtype = np.dtype({"names": [name for name in names],
                      "formats": [item[1:] for item in dtype_fields],
                      "offsets": offsets, "itemsize": point_step})
    points = np.memmap(path, mode="r", dtype=dtype, offset=fields["data_offset"],
                       shape=(fields["points"],))
    return points


def xyz(points):
    return np.column_stack((points["x"], points["y"], points["z"])).astype(np.float32, copy=False)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--map-pcd", type=Path, required=True)
    parser.add_argument("--geometry-csv", type=Path, required=True)
    parser.add_argument("--projection-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    map_points = load_binary_xyz(args.map_pcd)
    map_xyz = xyz(map_points)[::100]
    rows = list(csv.DictReader(args.geometry_csv.open(newline="")))
    if len(rows) != 6:
        raise ValueError("expected exactly six frozen A/B cases")
    roles = []
    for row in rows:
        if row["role"] not in roles:
            roles.append(row["role"])
    if len(roles) != 3:
        raise ValueError("expected three frozen frame roles")

    projections = {}
    stats = {}
    for row in rows:
        key = (row["role"], row["hypothesis"])
        path = args.projection_dir / f"projection_{row['role']}_{row['hypothesis']}.pcd"
        projected = xyz(load_binary_xyz(path))
        projections[key] = projected
        stats[key] = (float(row["nn_median_m"]), float(row["nn_p90_m"]))

    dimensions = ((0, 1, "X", "Y", "XY"), (0, 2, "X", "Z", "XZ"), (1, 2, "Y", "Z", "YZ"))
    all_overlay = np.vstack(list(projections.values()))
    fig, axes = plt.subplots(3, 3, figsize=(17, 15), constrained_layout=True)
    colors = {"A_optical": "#d95f02", "B_base": "#1b9e77"}
    shared_limits = {}
    for dim0, dim1, _, _, plane in dimensions:
        lower = np.percentile(all_overlay, 0.5, axis=0) - 4.0
        upper = np.percentile(all_overlay, 99.5, axis=0) + 4.0
        shared_limits[plane] = (lower, upper)
    global_lower = np.min(np.vstack([limits[0] for limits in shared_limits.values()]), axis=0)
    global_upper = np.max(np.vstack([limits[1] for limits in shared_limits.values()]), axis=0)
    map_mask = np.all((map_xyz >= global_lower) & (map_xyz <= global_upper), axis=1)
    local_map = map_xyz[map_mask]
    for column, role in enumerate(roles):
        A = projections[(role, "A_optical")]
        B = projections[(role, "B_base")]
        for row_index, (dim0, dim1, name0, name1, plane) in enumerate(dimensions):
            axis = axes[row_index, column]
            lower, upper = shared_limits[plane]
            if local_map.size:
                axis.scatter(local_map[:, dim0], local_map[:, dim1], s=0.12, c="#b8b8b8",
                             alpha=0.22, linewidths=0, rasterized=True, label="prior map sample")
            for hypothesis, data in (("A_optical", A), ("B_base", B)):
                median, p90 = stats[(role, hypothesis)]
                axis.scatter(data[:, dim0], data[:, dim1], s=2.0, c=colors[hypothesis],
                             alpha=0.70, linewidths=0,
                             label=f"{hypothesis}: med {median:.2f} m / P90 {p90:.2f} m")
            axis.set_xlim(lower[dim0], upper[dim0])
            axis.set_ylim(lower[dim1], upper[dim1])
            axis.set_aspect("equal", adjustable="box")
            axis.grid(True, linewidth=0.35, alpha=0.4)
            axis.set_xlabel(f"{name0} [m]")
            axis.set_ylabel(f"{name1} [m]")
            if row_index == 0:
                axis.set_title(f"{role}\n{plane}")
            if column == 0:
                axis.text(0.02, 0.98, plane, transform=axis.transAxes, va="top", ha="left",
                          fontsize=9, bbox={"facecolor": "white", "alpha": 0.75, "edgecolor": "none"})
            if row_index == 0 and column == 0:
                axis.legend(loc="lower left", fontsize=7, markerscale=2, framealpha=0.9)
    fig.suptitle("Newer College 2020: official optical-axis vs Base-axis GT interpretation\n"
                 "Fixed scans; raw PCD XYZ, same preprocessing; shared axes across frames; no registration",
                 fontsize=14)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=180)
    print(f"saved {args.output}; sampled_map_points={len(local_map)} of {len(map_xyz)}")


if __name__ == "__main__":
    main()
