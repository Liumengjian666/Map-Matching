"""Observe and compare frozen candidate geometry without scores, labels or point selection."""
import argparse
import csv
from itertools import combinations
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

UNKNOWN, COMMON, DISCRIMINATIVE = 0, 1, 2
METHODS = ("GEO0_NEAREST", "GEO1_VOXEL", "GEO2_NORMAL", "GEO3_ROBUST_NORMAL")
CROP_M = 8.0
MIN_RANGE_M = 0.4
RAY_SUPPORT_M = 0.075
SCAN_RANGE_GATE_M = 0.30
VOXEL_M = 0.10
PLANE_RADIUS_M = 0.22
PLANE_NEIGHBORS = 24
PLANE_MIN_SUPPORT = 8
PLANE_EIGEN_MIN_M2 = 0.015 ** 2
PLANE_PLANARITY_MAX = 0.25
NORMAL_ANGLE_COS = np.cos(np.deg2rad(30.0))
INCIDENCE_COS_MIN = 0.30
EQUIV_M = 0.03
DIFFERENCE_M = 0.05
MAP_NOISE_FLOOR_M = 0.015


def transform(points, pose):
    return points @ pose[:3, :3].T + pose[:3, 3]


def validate(source, world, nominal, poses):
    if source.ndim != 2 or source.shape[1] != 3 or world.ndim != 2 or world.shape[1] != 3:
        raise ValueError("source and full map must be Nx3 XYZ in meters")
    if nominal.ndim != 3 or nominal.shape[1:] != (4, 4) or len(nominal) < 2:
        raise ValueError("nominal must contain two or more candidate matrices")
    if poses.shape != (len(nominal), 26, 4, 4):
        raise ValueError("perturbations must have 26 frozen transforms per candidate")
    if not all(np.isfinite(array).all() for array in (source, world, nominal, poses)):
        raise ValueError("non-finite geometry input")
    if not np.allclose(poses[:, [0, 13]], nominal[:, None], rtol=0, atol=1e-7):
        raise ValueError("both perturbation scales must start at the nominal candidate")
    for matrices in (nominal, poses.reshape(-1, 4, 4)):
        rotation = matrices[:, :3, :3]
        if not (np.allclose(matrices[:, 3], (0, 0, 0, 1), atol=1e-8)
                and np.allclose(rotation.transpose(0, 2, 1) @ rotation, np.eye(3), atol=2e-6)
                and np.linalg.det(rotation).min() > 0.999):
            raise ValueError("candidate maps must use proper rigid transforms")


def first_intersections(cloud, ray_unit, radius):
    """Earliest intersection of the measured lines and discrete map support spheres."""
    depth = np.full(len(ray_unit), np.inf)
    closest = np.full(len(ray_unit), -1, dtype=np.int64)
    if not len(cloud) or not len(ray_unit):
        return depth, closest
    radius = np.broadcast_to(radius, len(cloud))
    distance = np.linalg.norm(cloud, axis=1)
    if np.any(radius >= distance):
        raise ValueError("map support sphere cannot contain the ray origin")
    angular_sine = radius / distance
    chord = angular_sine * np.sqrt(2.0 / (1.0 + np.sqrt(1.0 - angular_sine ** 2)))
    matches = cKDTree(ray_unit).query_ball_point(cloud / distance[:, None], chord)
    counts = np.fromiter(map(len, matches), dtype=int, count=len(cloud))
    if not counts.sum():
        return depth, closest
    cloud_index = np.repeat(np.arange(len(cloud)), counts)
    ray_index = np.concatenate([indices for indices in matches if len(indices)]).astype(np.int64)
    projection = np.einsum("ij,ij->i", cloud[cloud_index], ray_unit[ray_index])
    cross = np.maximum(distance[cloud_index] ** 2 - projection ** 2, 0)
    intersection = projection - np.sqrt(np.maximum(radius[cloud_index] ** 2 - cross, 0))
    order = np.lexsort((cloud_index, intersection, ray_index))
    ray_sorted = ray_index[order]
    first = np.r_[True, ray_sorted[1:] != ray_sorted[:-1]]
    chosen = order[first]
    depth[ray_index[chosen]] = intersection[chosen]
    closest[ray_index[chosen]] = cloud_index[chosen]
    return depth, closest


def make_view(world, center, perturbations, ray_unit, ray_range):
    local = transform(world, np.linalg.inv(center))
    radius = np.linalg.norm(local, axis=1)
    # Geometry comparison is under the same scope on both sides. Points closer
    # than the admitted range only veto; a cropped near surface cannot confirm visibility.
    cloud = local[(radius >= MIN_RANGE_M) & (radius <= CROP_M)]
    near_cloud = local[(radius >= RAY_SUPPORT_M + 1.e-9) & (radius < MIN_RANGE_M)]
    origin_has_map_sample = bool(np.any(radius <= RAY_SUPPORT_M))
    front, ids = first_intersections(cloud, ray_unit, RAY_SUPPORT_M)
    too_near, _ = first_intersections(near_cloud, ray_unit, RAY_SUPPORT_M)
    pert_to_nominal = np.linalg.inv(perturbations) @ center
    # Pose transforms map points in nominal coordinates to the perturbed
    # sensor coordinates. The outer shell is a conservative swept-sphere
    # superset for these 26 samples, not an estimate of sensor covariance.
    risk_cloud = local[(radius > RAY_SUPPORT_M) & (radius <= CROP_M)]
    earliest, radial_crop_padding = [], []
    for scale in np.split(pert_to_nominal, 2):
        displacement = np.zeros(len(risk_cloud))
        for pose in scale:
            displacement = np.maximum(displacement, np.linalg.norm(transform(risk_cloud, pose) - risk_cloud, axis=1))
        swept = RAY_SUPPORT_M + displacement
        unresolved_origin = origin_has_map_sample or bool(np.any(swept >= np.linalg.norm(risk_cloud, axis=1)))
        if unresolved_origin:
            possible_front = np.full(len(ray_unit), -np.inf)
        else:
            possible_front, _ = first_intersections(risk_cloud, ray_unit, swept)
        earliest.append(possible_front)
        radial_crop_padding.append(np.linalg.norm(scale[:, :3, 3], axis=1).max())
    good = (ids >= 0) & (too_near > front) & (np.abs(front - ray_range) <= SCAN_RANGE_GATE_M)
    if origin_has_map_sample:
        good[:] = False
    return dict(map=cloud, front=front, ids=ids, good=good, earliest=earliest, radial_padding=radial_crop_padding,
                perturbations=pert_to_nominal,
                occupied={tuple(x) for x in np.floor(cloud / VOXEL_M).astype(int)})


def local_planes(view, ray_unit):
    cloud, ids = view["map"], view["ids"]
    count = len(ids)
    normal = np.full((count, 3), np.nan)
    mean = np.full((count, 3), np.nan)
    sigma = np.full(count, np.nan)
    support = np.zeros(count, dtype=int)
    good = np.zeros(count, dtype=bool)
    indices = np.flatnonzero(view["good"])
    if not len(indices):
        return dict(normal=normal, mean=mean, sigma=sigma, support=support, good=good)

    tree = cKDTree(cloud)
    max_neighbors = min(PLANE_NEIGHBORS, len(cloud))
    if max_neighbors < PLANE_MIN_SUPPORT:
        return dict(normal=normal, mean=mean, sigma=sigma, support=support, good=good)
    for row, near in zip(indices, tree.query(cloud[ids[indices]], k=max_neighbors)[1]):
        sample = cloud[near]
        center_distance = np.linalg.norm(sample - cloud[ids[row]], axis=1)
        sample = sample[center_distance <= PLANE_RADIUS_M]
        support[row] = len(sample)
        if len(sample) < PLANE_MIN_SUPPORT:
            continue
        centroid = sample.mean(axis=0)
        x = sample - centroid
        eigenvalue, eigenvector = np.linalg.eigh(x.T @ x / (len(x) - 1))
        if eigenvalue[1] < PLANE_EIGEN_MIN_M2 or eigenvalue[0] / eigenvalue[1] > PLANE_PLANARITY_MAX:
            continue
        direction = eigenvector[:, 0]
        if direction @ ray_unit[row] > 0:
            direction *= -1
        normal[row] = direction
        mean[row] = centroid
        sigma[row] = np.sqrt(max(eigenvalue[0], 0.0))
        good[row] = abs(direction @ ray_unit[row]) >= INCIDENCE_COS_MIN
    return dict(normal=normal, mean=mean, sigma=sigma, support=support, good=good)


def plane_intersections(plane, directions, ray_range, transform_set):
    mapped_normals = np.einsum("sij,nj->sni", transform_set[:, :3, :3], plane["normal"])
    mapped_means = np.einsum("sij,nj->sni", transform_set[:, :3, :3], plane["mean"])
    mapped_means += transform_set[:, None, :3, 3]
    numerator = np.einsum("sni,sni->sn", mapped_normals, mapped_means)
    denominator = np.einsum("sni,ni->sn", mapped_normals, directions)
    depth = np.divide(numerator, denominator, out=np.full_like(numerator, np.nan),
                      where=np.abs(denominator) >= INCIDENCE_COS_MIN)
    normal_hull = np.linalg.norm(depth[:, :, None] * directions[None] - mapped_means, axis=2)
    supported = (np.abs(depth - ray_range) <= SCAN_RANGE_GATE_M) & (normal_hull <= PLANE_RADIUS_M)
    return depth, mapped_normals, supported


def assign_interval(low, high, observable):
    positive = low >= DIFFERENCE_M
    negative = high <= -DIFFERENCE_M
    equivalent = (low >= -EQUIV_M) & (high <= EQUIV_M)
    return np.where(observable & (positive | negative), DISCRIMINATIVE,
                    np.where(observable & equivalent, COMMON, UNKNOWN)).astype(np.uint8)


def occupancy(points, voxels):
    return np.fromiter((tuple(idx) in voxels for idx in np.floor(points / VOXEL_M).astype(int)),
                       dtype=bool, count=len(points))


def common_incidence(pa, pb, rays):
    normal = pa["normal"] + pb["normal"]
    norm = np.linalg.norm(normal, axis=1)
    projection = np.einsum("ni,ni->n", normal, rays)
    return np.divide(projection, norm, out=np.full(len(rays), np.nan), where=norm > 0)


def robust_interval(a, b, pa, pb, rays, distances, scale, plain_observable):
    begin = 13 * (scale - 1)
    scenario = slice(begin, begin + 13)
    da, na, a_support = plane_intersections(pa, rays, distances, a["perturbations"][scenario])
    db, nb, b_support = plane_intersections(pb, rays, distances, b["perturbations"][scenario])
    incidence = common_incidence(pa, pb, rays)
    d1 = (da.min(axis=0) - db.max(axis=0)) * incidence
    d2 = (da.max(axis=0) - db.min(axis=0)) * incidence
    halfwidth = np.maximum(MAP_NOISE_FLOOR_M, 1.5 * (pa["sigma"] + pb["sigma"]))
    low, high = np.minimum(d1, d2) - halfwidth, np.maximum(d1, d2) + halfwidth
    reliable_angle = np.ones(len(rays), dtype=bool)
    for direction_a in na:
        agreement = np.einsum("ni,sni->sn", direction_a, nb)
        reliable_angle &= np.all(agreement >= NORMAL_ANGLE_COS, axis=0)
    scope_a = a["radial_padding"][scale - 1]
    scope_b = b["radial_padding"][scale - 1]
    risk_a = a["earliest"][scale - 1]
    risk_b = b["earliest"][scale - 1]
    outer_margin = SCAN_RANGE_GATE_M + RAY_SUPPORT_M + max(scope_a, scope_b)
    visible = ((risk_a >= da.max(axis=0) - SCAN_RANGE_GATE_M)
               & (risk_b >= db.max(axis=0) - SCAN_RANGE_GATE_M)
               & (distances <= CROP_M - outer_margin))
    observable = plain_observable & a_support.all(axis=0) & b_support.all(axis=0) & reliable_angle & visible
    return low, high, observable


def compare_pair(a, b, pa, pb, rays, distances, eligible):
    base = eligible & a["good"] & b["good"]
    sample = len(base)
    scores = np.zeros((sample, 4))
    classes = np.zeros((sample, 4), dtype=np.uint8)
    observable = np.column_stack((base, base, np.zeros_like(base), np.zeros_like(base)))
    if base.any():
        qa = a["map"][a["ids"][base]]
        qb = b["map"][b["ids"][base]]
        near_ab = cKDTree(b["map"]).query(qa)[0]
        near_ba = cKDTree(a["map"]).query(qb)[0]
        near = np.maximum(near_ab, near_ba)
        scores[base, 0] = near
        classes[base, 0] = np.where(near <= EQUIV_M, COMMON,
                                    np.where(near >= DIFFERENCE_M, DISCRIMINATIVE, UNKNOWN))
        in_b, in_a = occupancy(qa, b["occupied"]), occupancy(qb, a["occupied"])
        scores[base, 1] = 1.0 - 0.5 * (in_b.astype(float) + in_a.astype(float))
        classes[base, 1] = np.where(in_b & in_a & (near <= EQUIV_M), COMMON,
                                    np.where(~in_b & ~in_a & (near >= DIFFERENCE_M),
                                             DISCRIMINATIVE, UNKNOWN))

    normal_agreement = np.einsum("ni,ni->n", pa["normal"], pb["normal"]) >= NORMAL_ANGLE_COS
    plain_support = base & pa["good"] & pb["good"] & normal_agreement
    da, _, supports_a = plane_intersections(pa, rays, distances, a["perturbations"][:1])
    db, _, supports_b = plane_intersections(pb, rays, distances, b["perturbations"][:1])
    plain_support &= supports_a[0] & supports_b[0]
    incidence = common_incidence(pa, pb, rays)
    with np.errstate(invalid="ignore"):
        raw = (a["front"] - b["front"]) * incidence
        corrected = (da[0] - db[0]) * incidence
    raw = np.where(plain_support, raw, np.nan)
    corrected = np.where(plain_support, corrected, np.nan)
    observable[:, 2] = plain_support
    scores[plain_support, 2] = np.abs(corrected[plain_support])
    classes[:, 2] = assign_interval(corrected, corrected, plain_support)
    lows, highs, robust_support, robust_classes = [], [], [], []
    for scale in (1, 2):
        low, high, support = robust_interval(a, b, pa, pb, rays, distances, scale, plain_support)
        lows.append(np.where(support, low, np.nan))
        highs.append(np.where(support, high, np.nan))
        robust_support.append(support)
        robust_classes.append(assign_interval(low, high, support))
    observable[:, 3] = robust_support[0]
    scores[robust_support[0], 3] = np.maximum.reduce((
        lows[0][robust_support[0]], -highs[0][robust_support[0]], np.zeros(robust_support[0].sum())))
    classes[:, 3] = robust_classes[0]
    return dict(scores=scores, classes=classes, observable=observable, raw_normal=raw, corrected_normal=corrected,
                normal_low=np.stack(lows), normal_high=np.stack(highs),
                robust_observable=np.stack(robust_support), robust_classes=np.stack(robust_classes))


def compute(source, world, nominal, poses):
    validate(source, world, nominal, poses)
    distances = np.linalg.norm(source, axis=1)
    eligible = (distances >= MIN_RANGE_M) & (distances <= CROP_M)
    with np.errstate(invalid="ignore", divide="ignore"):
        rays = np.divide(source, distances[:, None], out=np.tile([0., 0., 1.], (len(source), 1)),
                         where=distances[:, None] > 0)
    views = [make_view(world, center, perturbations, rays, distances)
             for center, perturbations in zip(nominal, poses)]
    planes = [local_planes(view, rays) for view in views]
    pairs = np.asarray(list(combinations(range(len(views)), 2)))
    per_pair = [compare_pair(views[a], views[b], planes[a], planes[b], rays, distances, eligible)
                for a, b in pairs]
    detail = {"pair_" + k: np.stack([d[k] for d in per_pair]) for k in per_pair[0]}
    detail["pairs"] = pairs
    detail["eligible_range"] = eligible
    detail["ray_ranges"] = distances
    detail["first_intersection_depths"] = np.stack([v["front"] for v in views])
    detail["first_intersection_support"] = np.stack([v["good"] & eligible for v in views])
    detail["local_plane_points"] = np.stack([p["support"] for p in planes])
    detail["observable"] = detail["pair_observable"].all(axis=0)
    detail["scores"] = np.where(detail["observable"], detail["pair_scores"].max(axis=0), 0.)
    classified = detail["pair_classes"]
    detail["classes"] = np.where(detail["observable"] & (classified == DISCRIMINATIVE).any(axis=0), DISCRIMINATIVE,
                                 np.where(detail["observable"] & (classified == COMMON).all(axis=0), COMMON, UNKNOWN))
    detail["class_names"] = np.array(("UNKNOWN", "COMMON", "DISCRIMINATIVE"), dtype="U16")
    detail["method_names"] = np.array(METHODS, dtype="U24")
    return detail, views


def result_rows(sample, data):
    def one_group(label, names, scores, classes, observable):
        for slot, name in enumerate(names):
            score, observed, labels = scores[:, slot], observable[:, slot], classes[:, slot]
            scores_observed = score[observed]
            yield dict(sample_id=sample, pair=label, method=name, points=len(score), observable=int(observed.sum()),
                       common=int((labels == COMMON).sum()), discriminative=int((labels == DISCRIMINATIVE).sum()),
                       unknown=int((labels == UNKNOWN).sum()),
                       observable_unknown=int((observed & (labels == UNKNOWN)).sum()),
                       support_fraction=float(observed.mean()),
                       observable_score_median=float(np.median(scores_observed)) if len(scores_observed) else "")
    yield from one_group("aggregate", METHODS, data["scores"], data["classes"], data["observable"])
    for i, (a, b) in enumerate(data["pairs"]):
        label = f"{a}-{b}"
        yield from one_group(label, METHODS, data["pair_scores"][i], data["pair_classes"][i],
                             data["pair_observable"][i])
        score_scale_two = np.maximum.reduce((
            np.nan_to_num(data["pair_normal_low"][i, 1]), -np.nan_to_num(data["pair_normal_high"][i, 1]),
            np.zeros(len(data["scores"]))))
        yield from one_group(label, ("GEO3_ROBUST_NORMAL_SCALE2",), score_scale_two[:, None],
                             data["pair_robust_classes"][i, 1, :, None], data["pair_robust_observable"][i, 1, :, None])


COLORS = np.array(((142, 147, 153), (28, 148, 113), (217, 87, 60)), dtype=np.uint32)


def write_pcd(path, points, classes):
    colors = COLORS[classes]
    packed = ((colors[:, 0] << 16) | (colors[:, 1] << 8) | colors[:, 2]).astype("<u4").view("<f4")
    layout = np.column_stack((np.asarray(points, dtype="<f4"), packed))
    header = (
        "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n"
        f"WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA binary\n")
    with path.open("xb") as stream:
        stream.write(header.encode())
        stream.write(layout.tobytes())


def render(sample, source, data, views, output):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D

    panels = len(views) + len(METHODS)
    columns = min(panels, 4)
    rows = (panels + columns - 1) // columns
    fig, axes = plt.subplots(rows, columns, figsize=(4.1 * columns, 4.2 * rows), constrained_layout=True)
    for i, axis in enumerate(axes.flat[:len(views)]):
        cloud = views[i]["map"]
        axis.scatter(cloud[:, 0], cloud[:, 2], s=0.45, c="#8a9198", alpha=0.35, rasterized=True)
        measured = source[data["first_intersection_support"][i]]
        axis.scatter(measured[:, 0], measured[:, 2], s=2, c="#087e92", alpha=0.8, rasterized=True)
        axis.set_title(f"Candidate {i} | compatible rays {len(measured):,}")
    for slot, axis in enumerate(axes.flat[len(views):panels]):
        labels = data["classes"][:, slot]
        for label in (UNKNOWN, COMMON, DISCRIMINATIVE):
            points = source[labels == label]
            axis.scatter(points[:, 0], points[:, 2], s=2.8 if label else 1.1,
                         color=COLORS[label] / 255, alpha=0.88 if label else 0.50, rasterized=True)
        axis.set_title(METHODS[slot].replace("_", " ") +
                       f" | C {(labels == COMMON).sum()} / D {(labels == DISCRIMINATIVE).sum()}")
    for axis in axes.flat[:panels]:
        axis.set(aspect="equal", xlabel="Sensor x (m)", ylabel="Sensor z (m)",
                 xlim=(-4, 4), ylim=(-0.35, 7.9))
    for axis in axes.flat[panels:]:
        axis.set_axis_off()
    axes.flat[panels - 1].legend(handles=[Line2D([], [], marker="o", linestyle="None", color=COLORS[i]/255, label=label)
                                  for i, label in enumerate(("Unknown", "Common", "Discriminative"))],
                        loc="lower left", fontsize=8)
    fig.suptitle(sample + " | Geometric evidence in candidate sensor frames")
    fig.savefig(output / (sample + "_geometry_classification.png"), dpi=155)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("inputs", type=Path, nargs="+")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    rows = []
    for path in args.inputs:
        sample = path.stem
        with np.load(path, allow_pickle=False) as file:
            source, world, nominal, poses = [np.asarray(file[k], dtype=float)
                                             for k in ("source", "map", "nominal", "poses")]
        data, views = compute(source, world, nominal, poses)
        target = args.output / f"geometry_{sample}.npz"
        with target.open("xb") as stream:
            np.savez_compressed(stream, **data)
        rows.extend(result_rows(sample, data))
        if sample == "H307":
            for slot, method in enumerate(METHODS):
                write_pcd(args.output / f"{sample}_{method}_classes.pcd", source, data["classes"][:, slot])
            render(sample, source, data, views, args.output)
    with (args.output / "geometry_difference_results.csv").open("x", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


if __name__ == "__main__":
    main()
