"""DeCOD core paper reimplementation and shared-geometry candidate adapter."""
from dataclasses import dataclass
from pathlib import Path
import time

import numpy as np
from scipy.ndimage import gaussian_filter1d, maximum_filter
from scipy.signal import find_peaks
from scipy.sparse import coo_matrix
from scipy.sparse.linalg import lsqr
from scipy.spatial import cKDTree

AXIAL = .30
HALF = 2.4
NZ, NP = 16, 256
MAX_SHIFT = .45
MIN_INLIERS = 9
PATCH_Z, PATCH_P = np.meshgrid(np.arange(-1, 2), np.arange(-4, 5), indexing="ij")
PATCH_WEIGHT = np.exp(-.5 * ((PATCH_Z / 1.) ** 2 + (PATCH_P / 2.) ** 2)).ravel()


def voxel(points, leaf=.055):
    if not len(points):
        return points
    _, label, count = np.unique(np.floor(points / leaf).astype(int), axis=0, return_inverse=True,
                                return_counts=True)
    return np.stack([np.bincount(label, weights=points[:, j]) / count for j in range(3)], axis=1)


def local(world, index, pose, radius=6.5):
    loc = world[index.query_ball_point(pose[:3, 3], radius)]
    return (loc - pose[:3, 3]) @ pose[:3, :3]


@dataclass
class Section:
    # Columns are axial, projected-up, then the closing transverse direction.
    basis: np.ndarray
    low: np.ndarray
    high: np.ndarray
    elongation: float

    @classmethod
    def fit(cls, points, up, axis_hint=None):
        up = up / np.linalg.norm(up)
        local = points[np.linalg.norm(points, axis=1) < 6]
        if len(local) < 50:
            raise ValueError("insufficient contour samples")
        horizontal = local - (local @ up)[:, None] * up
        val, vec = np.linalg.eigh(np.cov(horizontal.T))
        axis = vec[:, -1]
        if axis_hint is not None:
            axis = axis_hint - (axis_hint @ up) * up
            if np.linalg.norm(axis) < .1:
                raise ValueError("weak direction has no horizontal corridor component")
            axis /= np.linalg.norm(axis)
        axis *= 1 if axis[np.argmax(abs(axis))] >= 0 else -1
        e1 = up - (up @ axis) * axis
        e1 /= np.linalg.norm(e1)
        basis = np.column_stack((axis, e1, np.cross(axis, e1)))
        yz = (local @ basis)[:, 1:]
        low, high = np.quantile(yz, [.1, .9], axis=0)
        if min(high - low) < .2:
            raise ValueError("degenerate transverse contour")
        spectrum = np.linalg.eigvalsh(np.cov(local.T))
        return cls(basis, low, high, spectrum[-1] / max(1e-9, spectrum[-2]))

    def project(self, points):
        x, h, y = (points @ self.basis).T
        h0, y0 = self.low
        h1, y1 = self.high
        dh, dy = self.high - self.low
        clipped_h = np.clip(h, h0, h1)
        clipped_y = np.clip(y, y0, y1)
        signed = np.stack((h - h1, y - y1, h0 - h, y0 - y), axis=1)
        tangential = np.stack((y - clipped_y, h - clipped_h, y - clipped_y, h - clipped_h), axis=1)
        side = np.argmin(signed ** 2 + tangential ** 2, axis=1)
        arclength = np.stack((clipped_y - y0, dy + h1 - clipped_h,
                              dy + dh + y1 - clipped_y, 2 * dy + dh + clipped_h - h0), axis=1)
        span = 2 * (dh + dy)
        phase = (arclength[np.arange(len(side)), side] - .5 * dy) % span / span
        offset = signed[np.arange(len(side)), side]
        norm = np.array(((1, 0), (0, 1), (-1, 0), (0, -1)), dtype=float)[side]
        return x, phase, offset, norm


def landmarks(projected):
    x, _, delta, _ = projected
    step = .01
    edge = np.arange(-5.5, 5.51, step)
    index = np.searchsorted(edge, x) - 1
    valid = (index >= 0) & (index < len(edge) - 1) & (abs(delta) <= .30)
    count = np.bincount(index[valid], minlength=len(edge) - 1)
    raw = np.bincount(index[valid], weights=abs(delta[valid]), minlength=len(count))
    if np.count_nonzero(count) < 10:
        return np.array([])
    good = np.flatnonzero(count)
    signal = np.interp(np.arange(len(count)), good, raw[good] / count[good])
    dog = np.abs(gaussian_filter1d(signal, .025 / step) - gaussian_filter1d(signal, .10 / step))
    dog[gaussian_filter1d(count.astype(float), .10 / step) < .4] = 0.
    peaks, _ = find_peaks(dog, distance=round(.8 / step), prominence=.002)
    peaks = peaks[(abs(edge[peaks]) <= 4.5)]
    return edge[peaks] + step / 2


@dataclass
class Chart:
    raw: np.ndarray
    normalized: np.ndarray
    base: np.ndarray
    points: np.ndarray


def chart(projected, points, center):
    x, phase, offset, normal = projected
    z = np.floor((x - center + HALF) / AXIAL).astype(int)
    p = np.floor(phase * NP).astype(int) % NP
    keep = (z >= 0) & (z < NZ) & (abs(offset) <= .30)
    index = z[keep] * NP + p[keep]
    count = np.bincount(index, minlength=NZ * NP)
    valid = np.flatnonzero(count)
    values = np.bincount(index, weights=offset[keep], minlength=len(count))[valid] / count[valid]
    iz, ip = np.divmod(valid, NP)
    cell_points = np.full((NZ * NP, 3), np.nan)
    arr = np.full(NZ * NP, np.nan)
    fit = arr.copy()
    normed = arr.copy()
    if len(valid):
        for j in range(3):
            cell_points[valid, j] = np.bincount(index, weights=points[keep, j],
                                               minlength=len(count))[valid] / count[valid]
        mean_normals = np.stack([np.bincount(index, weights=v, minlength=len(count))[valid] /
                                 count[valid] for v in normal[keep].T], axis=1)
        # Eq. (2). Sparse least squares solves the gauge-ambiguous additive model.
        zc = (iz + .5) * AXIAL - HALF
        cols = np.column_stack((ip, NP + iz, np.full(len(ip), NP + NZ),
                                np.full(len(ip), NP + NZ + 1)))
        design = coo_matrix((np.column_stack((np.ones((len(ip), 2)), zc[:, None] *
                                             mean_normals)).ravel(),
                             (np.repeat(np.arange(len(ip)), 4), cols.ravel())),
                            shape=(len(ip), NP + NZ + 2)).tocsr()
        fit[valid] = design @ lsqr(design, values, atol=1e-7, btol=1e-7, iter_lim=250)[0]
        arr[valid] = values
        normed[valid] = values - fit[valid]
    return Chart(arr.reshape(NZ, NP), normed.reshape(NZ, NP), fit.reshape(NZ, NP),
                 cell_points.reshape(NZ, NP, 3))


def patches(arr):
    observed = np.isfinite(arr)
    activity = np.where(observed, abs(arr), -np.inf)
    salient = activity == maximum_filter(activity, (3, 5), mode=("constant", "wrap"),
                                        cval=-np.inf)
    z, p = np.nonzero(salient & observed)
    keep = (z >= 1) & (z < NZ - 1)
    z, p = z[keep], p[keep]
    vals = arr[z[:, None] + PATCH_Z.ravel(), (p[:, None] + PATCH_P.ravel()) % NP]
    keep = np.isfinite(vals).sum(axis=1) >= 9
    return z[keep], p[keep], vals[keep]


def patch_matches(query, other, reverse=False, mode="normalized"):
    q = getattr(query, mode)
    t = getattr(other, mode)
    pts = other.points
    if reverse:
        t, pts = t[::-1, ::-1], pts[::-1, ::-1]
    z, p, a = patches(q)
    zz, pp, b = patches(t)
    if not len(a) or not len(b):
        return 0., np.empty((0, 2, 3)), np.nan
    vals = np.zeros((len(a), len(b)))
    weights = vals.copy()
    support = np.zeros_like(vals, dtype=int)
    for column, w in enumerate(PATCH_WEIGHT):
        delta = a[:, column, None] - b[None, :, column]
        valid = np.isfinite(delta)
        vals += w * np.where(valid, delta ** 2, 0.)
        weights += w * valid
        support += valid
    dist = np.sqrt(np.divide(vals, np.maximum(weights, 1e-12)))
    circ = abs(p[:, None] - pp[None, :])
    spatial = (abs(z[:, None] - zz[None, :]) <= 1) & (np.minimum(circ, NP-circ) <= 8)
    dist[~spatial | (support < 9)] = np.inf
    k = np.argmin(dist, axis=1)
    i = np.arange(len(k))
    unique_a = (dist == dist[i, k, None]).sum(axis=1) == 1
    d_for_b = np.min(dist, axis=0)
    unique_b = (dist == d_for_b).sum(axis=0) == 1
    keep = (np.argmin(dist, axis=0)[k] == i) & unique_a & unique_b[k] & np.isfinite(dist[i, k])
    lhs = query.points[z[keep], p[keep]]
    rhs = pts[zz[k[keep]], pp[k[keep]]]
    dif = rhs - lhs
    if not len(dif):
        return 0., np.empty((0, 2, 3)), np.nan
    center = np.median(dif, axis=0)
    good = (np.linalg.norm(dif - center, axis=1) <= .22) & (dist[i, k][keep] <= .08)
    return 2 * good.sum() / (len(a) + len(b)), np.stack((lhs[good], rhs[good]), axis=1), (
        float(np.median(dist[i, k][keep][good])) if good.any() else np.nan)


def assess(source, world, tree, poses, run=None, cloud_mask=None, axis_hint=None):
    start = time.perf_counter()
    query = voxel(source)
    views = [voxel(local(world, tree, pose)) for pose in poses]
    crop_s = time.perf_counter() - start
    section = Section.fit(views[0], poses[0][:3, :3].T @ np.array([0., 0., 1.]), axis_hint)
    obs = section.project(query)
    q_land = landmarks(obs)
    qualified = np.mean(abs(obs[2]) <= .18)
    q_ratio = np.linalg.eigvalsh(np.cov(query.T))
    q_ratio = q_ratio[-1] / max(1e-9, q_ratio[-2])
    queries = [(o, chart(obs, query, o)) for o in q_land]
    result = []
    for k, view in enumerate(views):
        tick = time.perf_counter()
        projected = section.project(view)
        t_land = landmarks(projected)
        extraction_s = time.perf_counter() - tick
        ranked = []
        for o, a in queries:
            for m in t_land[abs(t_land - o) <= MAX_SHIFT]:
                b = chart(projected, view, m)
                if cloud_mask is not None:
                    # Direct combination diagnostic. Use the same preselected M3C2
                    # source surface for all candidates, projected in the shared chart.
                    masked = chart(section.project(cloud_mask), cloud_mask, o).raw if len(cloud_mask) else np.full_like(a.raw, np.nan)
                    supported = np.isfinite(masked)
                    a_used = Chart(np.where(supported, a.raw, np.nan),
                                   np.where(supported, a.normalized, np.nan),
                                   np.where(supported, a.base, np.nan), a.points)
                else:
                    a_used = a
                for heading in (0, 180):
                    score, pairs, residual = patch_matches(a_used, b, heading == 180)
                    ranked.append((score, len(pairs), -float(abs(m-o)), o, m, heading, residual, pairs, a_used, b))
        ranked.sort(key=lambda item:item[:3], reverse=True)
        if not ranked:
            result.append(dict(candidate=k, cost=1., score=0., inliers=0, landmarks=len(t_land),
                               query_landmarks=len(q_land), elongation=q_ratio, contour_fraction=qualified,
                               eligible=False, reason="NO_COMPATIBLE_LANDMARK", crop_s=crop_s,
                               extract_s=extraction_s, match_s=time.perf_counter()-tick-extraction_s))
            continue
        score, n, _, o, m, heading, residual, pairs, a, b = ranked[0]
        unique = len(ranked) == 1 or (score > ranked[1][0] + 1e-9)
        raw, _, _ = patch_matches(a,b,heading == 180,"raw")
        base, _, _ = patch_matches(a,b,heading == 180,"base")
        eligible = n >= MIN_INLIERS and heading == 0 and unique and q_ratio >= 2 and qualified >= .90
        result.append(dict(candidate=k,cost=1-score,score=score,inliers=n,landmarks=len(t_land),
                           query_landmarks=len(q_land),elongation=q_ratio,contour_fraction=qualified,
                           eligible=bool(eligible),reason="VERIFIED" if eligible else "PAPER_OR_ASSOCIATION_GATE",
                           residual_m=residual,heading=heading,raw_score=raw,base_score=base,
                           axis_shift_m=float(m-o),axis=section.basis[:,0].tolist(),crop_s=crop_s,
                           extract_s=extraction_s,match_s=time.perf_counter()-tick-extraction_s))
        if run is not None:
            np.savez_compressed(Path(run)/f"landmark_{k}.npz",query=query,view=view,pairs=pairs,
                                query_raw=a.raw,query_detail=a.normalized,candidate_raw=b.raw,
                                candidate_detail=b.normalized,query_base=a.base,candidate_base=b.base,
                                axis=section.basis[:,0],query_center=o,map_center=m)
    return result


def select(result):
    # Retain all scores; a failed qualification at a higher score is not a
    # license to silently pick a lower-ranked candidate.
    ordered = sorted(result, key=lambda row: (-row["score"], row["candidate"]))
    if not ordered or not ordered[0]["eligible"]:
        return None
    if len(ordered) > 1 and ordered[0]["score"] <= ordered[1]["score"] + 1e-9:
        return None
    return ordered[0]
