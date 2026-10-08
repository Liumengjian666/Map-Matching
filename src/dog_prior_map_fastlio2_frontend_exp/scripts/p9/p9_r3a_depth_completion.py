"""Offline conservative metric depth; never modifies the P4 DIRECT frontend."""
import time

import numpy as np
from scipy.spatial import cKDTree

from p9_r3_visual_contract import frontend as p4

PARAMETERS = dict(max_neighbors=12, radius_px=16., min_neighbors=6,
    planarity_ratio_max=.02, residual_absolute_m=.05, residual_depth_fraction=.01,
    spread_absolute_m=.50, spread_depth_fraction=.10, ray_dot_min=.10,
    depth_lower_factor=.9, depth_upper_factor=1.1)


def project_visible(cloud, calibration):
    """Exactly P4 projection/z-buffer: nearest positive camera Z per pixel."""
    xyz = p4.xyz_array(cloud)
    transform = calibration["T_camera_lidar"]
    xyz = xyz @ transform[:3, :3].T + transform[:3, 3]
    xyz = xyz[np.isfinite(xyz).all(axis=1) & (xyz[:, 2] > 0)]
    uvh = xyz @ calibration["Krect"].T
    uv = uvh[:, :2] / uvh[:, 2:3]
    w, h = calibration["size"]
    valid = (uv[:, 0] >= 0) & (uv[:, 0] < w-1) & (uv[:, 1] >= 0) & (uv[:, 1] < h-1)
    uv, xyz = uv[valid], xyz[valid]
    if not len(uv):
        return uv, xyz
    pixels = np.rint(uv).astype(int)
    pixel_id = pixels[:, 1] * w + pixels[:, 0]
    order = np.lexsort((xyz[:, 2], pixel_id))
    _, first = np.unique(pixel_id[order], return_index=True)
    chosen = order[first]
    return uv[chosen], xyz[chosen]


def plane_point(feature, uv, xyz, inverse_k):
    """Fit one fixed local neighborhood; no trimming, GT, or second attempt."""
    diagnostic = dict(neighbor_count=len(xyz), radius_px="", planarity_ratio="",
        median_plane_residual_m="", depth_spread_m="", estimated_depth_m="",
        neighbor_depth_min_m="", neighbor_depth_max_m="", neighbor_depth_median_m="",
        ray_dot="", rejection="TOO_FEW_NEIGHBORS")
    if len(xyz):
        diagnostic["radius_px"] = float(np.max(np.linalg.norm(uv-feature, axis=1)))
        diagnostic.update(neighbor_depth_min_m=float(xyz[:,2].min()),
            neighbor_depth_max_m=float(xyz[:,2].max()),neighbor_depth_median_m=float(np.median(xyz[:,2])))
    if len(xyz) < PARAMETERS["min_neighbors"]:
        return None, diagnostic
    center = np.mean(xyz, axis=0)
    centered = xyz-center
    values, vectors = np.linalg.eigh(centered.T @ centered / len(xyz))
    total = float(np.sum(values))
    if not np.isfinite(values).all() or total <= 0:
        diagnostic["rejection"] = "DEGENERATE_PLANE"
        return None, diagnostic
    ratio = float(max(0., values[0]) / total)
    normal = vectors[:, 0]  # unit normal; eigenvalues ascending
    residual = float(np.median(np.abs(centered @ normal)))
    zmin, zmax, zmed = float(xyz[:, 2].min()), float(xyz[:, 2].max()), float(np.median(xyz[:, 2]))
    diagnostic.update(planarity_ratio=ratio, median_plane_residual_m=residual,
                      depth_spread_m=zmax-zmin)
    if ratio > PARAMETERS["planarity_ratio_max"]:
        diagnostic["rejection"] = "NONPLANAR"
        return None, diagnostic
    if residual > max(PARAMETERS["residual_absolute_m"], PARAMETERS["residual_depth_fraction"]*zmed):
        diagnostic["rejection"] = "PLANE_RESIDUAL"
        return None, diagnostic
    if zmax-zmin > max(PARAMETERS["spread_absolute_m"], PARAMETERS["spread_depth_fraction"]*zmed):
        diagnostic["rejection"] = "DEPTH_DISCONTINUITY"
        return None, diagnostic
    hull = p4.cv2.convexHull(np.asarray(uv, dtype=np.float32))
    if p4.cv2.contourArea(hull) <= 0 or p4.cv2.pointPolygonTest(hull, tuple(map(float, feature)), False) < 0:
        diagnostic["rejection"] = "OUTSIDE_CONVEX_HULL"
        return None, diagnostic
    ray = inverse_k @ np.r_[feature, 1.]
    dot = float(normal @ ray)
    diagnostic["ray_dot"] = abs(dot)
    if abs(dot) < PARAMETERS["ray_dot_min"]:
        diagnostic["rejection"] = "RAY_PARALLEL"
        return None, diagnostic
    point = ray * float(normal @ center) / dot
    if not np.isfinite(point).all() or point[2] <= 0:
        diagnostic["rejection"] = "INVALID_INTERSECTION"
        return None, diagnostic
    diagnostic["estimated_depth_m"] = float(point[2])
    if not PARAMETERS["depth_lower_factor"]*zmin <= point[2] <= PARAMETERS["depth_upper_factor"]*zmax:
        diagnostic["rejection"] = "INTERSECTION_DEPTH_BOUND"
        return None, diagnostic
    diagnostic["rejection"] = ""
    return point, diagnostic


def associate(cloud, features, calibration):
    """Return DIRECT and AUGMENTED arrays in original FB feature order."""
    started = time.perf_counter()
    uv, xyz = project_visible(cloud, calibration)
    timing = dict(projection_ms=(time.perf_counter()-started)*1000,
        direct_association_ms=0., neighbor_search_ms=0., plane_fitting_ms=0.,
        completion_ms=0.)
    count = len(features)
    direct = np.zeros(count, dtype=bool)
    points = np.full((count, 3), np.nan)
    records = [dict(feature_index=i, ref_u=float(p[0]), ref_v=float(p[1]),
        depth_source="MISSING", neighbor_count=0, radius_px="", planarity_ratio="",
        median_plane_residual_m="", depth_spread_m="", estimated_depth_m="",
        neighbor_depth_min_m="",neighbor_depth_max_m="",neighbor_depth_median_m="",
        ray_dot="", neighbor_indices="", rejection="NO_PROJECTED_POINTS") for i, p in enumerate(features)]
    if not len(uv):
        return direct, points[direct], direct.copy(), points[direct], records, timing
    tick = time.perf_counter()
    tree = cKDTree(uv)
    distance, index = tree.query(features, distance_upper_bound=2.)
    direct = np.isfinite(distance)
    inverse_k = np.linalg.inv(calibration["Krect"])
    rays = np.column_stack([features[direct], np.ones(direct.sum())]) @ inverse_k.T
    points[direct] = rays * xyz[index[direct], 2:3]
    for i in np.flatnonzero(direct):
        records[i].update(depth_source="DIRECT", neighbor_count=1, radius_px=float(distance[i]),
            estimated_depth_m=float(points[i, 2]), neighbor_depth_min_m=float(points[i,2]),
            neighbor_depth_max_m=float(points[i,2]),neighbor_depth_median_m=float(points[i,2]),
            neighbor_indices=str(index[i]), rejection="")
    timing["direct_association_ms"] = (time.perf_counter()-tick)*1000
    direct_points = points[direct].copy()
    good = direct.copy()
    tick = time.perf_counter()
    distances, indices = tree.query(features[~direct], k=PARAMETERS["max_neighbors"],
        distance_upper_bound=np.nextafter(PARAMETERS["radius_px"], np.inf))
    timing["neighbor_search_ms"] = (time.perf_counter()-tick)*1000
    tick = time.perf_counter()
    for feature_id, ds, ids in zip(np.flatnonzero(~direct), distances, indices):
        take = np.isfinite(ds) & (ds <= PARAMETERS["radius_px"])
        used = ids[take]
        point, diagnostic = plane_point(features[feature_id], uv[used], xyz[used], inverse_k)
        records[feature_id].update(diagnostic, neighbor_indices=";".join(map(str, used)))
        if point is not None:
            points[feature_id] = point
            good[feature_id] = True
            records[feature_id]["depth_source"] = "PLANE_COMPLETED"
    timing["plane_fitting_ms"] = (time.perf_counter()-tick)*1000
    timing["completion_ms"] = timing["neighbor_search_ms"]+timing["plane_fitting_ms"]
    return direct, direct_points, good, points[good], records, timing


def self_test():
    from types import SimpleNamespace
    k = np.array([[100., 0., 50.], [0., 100., 50.], [0., 0., 1.]])
    uv = np.array([[44,44], [50,44], [56,44], [44,56], [50,56], [56,56]], dtype=float)
    rays = np.c_[uv, np.ones(6)] @ np.linalg.inv(k).T
    xyz = rays*5.
    point, d = plane_point(np.array([50.,50.]), uv, xyz, np.linalg.inv(k))
    assert point is not None and np.max(abs(point-[0,0,5])) < 1e-12 and d["rejection"] == ""
    assert plane_point(np.array([80.,50.]), uv, xyz, np.linalg.inv(k))[1]["rejection"] == "OUTSIDE_CONVEX_HULL"
    # Construct an exactly planar tilted neighborhood whose depth span is too large.
    tilted = rays * (5./(1.-5.*rays[:,0:1]))
    assert plane_point(np.array([50.,50.]), uv, tilted, np.linalg.inv(k))[1]["rejection"] == "DEPTH_DISCONTINUITY"
    parallel = np.c_[np.full(6,.001), (uv[:,1]-50)*.05, 5+(uv[:,0]-50)*.01]
    assert plane_point(np.array([50.,50.]), uv, parallel, np.linalg.inv(k))[1]["rejection"] == "RAY_PARALLEL"
    assert plane_point(np.array([50.,50.]), uv[:5], xyz[:5], np.linalg.inv(k))[0] is None
    values = np.vstack([xyz, [0.,0.,3.], [0.,0.,7.], [0.,0.,-1.] ]).astype("<f4")
    cloud = SimpleNamespace(fields=[SimpleNamespace(name=n, datatype=7,count=1,offset=i*4) for i,n in enumerate("xyz")],
        is_bigendian=False, point_step=12, height=1,width=len(values), row_step=12*len(values),data=values.tobytes())
    calibration = dict(Krect=k,T_camera_lidar=np.eye(4),size=(100,100))
    features = np.array([[50.,50.],[50.,52.]], dtype=np.float32)
    old_good, old_points, _, _ = p4.associate_depth(cloud,features,calibration)
    direct, points, good, augmented, records, _ = associate(cloud,features,calibration)
    assert np.array_equal(direct,old_good) and np.array_equal(points,old_points)
    assert records[0]["depth_source"] == "DIRECT" and points[0,2] == 3.
    positions = np.searchsorted(np.flatnonzero(good), np.flatnonzero(direct))
    assert np.array_equal(augmented[positions], points), "completion replaced/reordered DIRECT"
    plane_cloud = SimpleNamespace(**dict(vars(cloud),width=6,row_step=72,data=xyz.astype("<f4").tobytes()))
    mixed = np.array([[44.,44.],[50.,50.]],dtype=np.float32)
    direct, points, good, augmented, records, _ = associate(plane_cloud,mixed,calibration)
    assert direct.tolist()==[True,False] and good.tolist()==[True,True]
    assert [r["depth_source"] for r in records]==["DIRECT","PLANE_COMPLETED"]
    assert np.array_equal(points[0],augmented[0]) and np.max(abs(augmented[1]-[0,0,5]))<1e-10
    sparse = plane_point(np.array([50.,50.]),uv[:2],xyz[:2],np.linalg.inv(k))[1]
    assert sparse["rejection"]=="TOO_FEW_NEIGHBORS" and sparse["radius_px"]>0
    empty = SimpleNamespace(**dict(vars(cloud),width=0,row_step=0,data=b""))
    assert not associate(empty,features,calibration)[2].any()
    print("P9_R3A_DIRECT_PLANE_DISCONTINUITY_HULL_RAY_SELF_TEST=PASS")


if __name__ == "__main__":
    self_test()
