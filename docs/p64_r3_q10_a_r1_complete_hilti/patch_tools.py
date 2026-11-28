"""Exact bounded fixed-correspondence solvers and reusable map normals."""
from pathlib import Path
import hashlib
import os
import tempfile

import numpy as np
from scipy.spatial import cKDTree


def least_squares(a, b, lower, upper):
    denominator = b @ b
    return float(np.clip(-float(a @ b) / denominator if denominator else 0., lower, upper))


def trunc_squared(a, b, lower, upper, tau):
    """Sweep the intervals with an unchanged loss-active set, including endpoints."""
    a = np.asarray(a, dtype=np.longdouble)
    b = np.asarray(b, dtype=np.longdouble)
    if a.ndim != 1 or a.shape != b.shape or not np.isfinite(a).all() or not np.isfinite(b).all():
        raise ValueError("expected finite paired residual coefficients")
    if not np.isfinite([lower, upper, tau]).all() or lower > upper or tau <= 0:
        raise ValueError("invalid bounds or truncation threshold")
    ref = np.longdouble(lower)
    span = np.longdouble(upper) - ref
    r = a + ref * b
    safe = b != 0
    entering = np.minimum((-tau-r[safe])/b[safe], (tau-r[safe])/b[safe])
    leaving = np.maximum((-tau-r[safe])/b[safe], (tau-r[safe])/b[safe])
    events = np.concatenate((entering, leaving))
    indices = np.tile(np.flatnonzero(safe), 2)
    sign = np.repeat([1, -1], len(entering))
    use = (events > 0) & (events < span)
    order = np.argsort(events[use], kind="stable")
    indices, sign, times = indices[use][order], sign[use][order], events[use][order]
    active = safe.copy()
    active[safe] = (entering <= 0) & (leaving > 0)
    square = b*b
    slope = r*b
    tau2 = np.longdouble(tau)**2
    loss_delta = r*r - tau2
    starts = np.r_[np.longdouble(0), times]
    ends = np.r_[times, span]
    h = np.cumsum(np.r_[square[active].sum(), sign*square[indices]])
    g = np.cumsum(np.r_[slope[active].sum(), sign*slope[indices]])
    constant = np.cumsum(np.r_[
        np.minimum(r[~active]**2, tau2).sum() + (r[active]**2).sum(),
        sign*loss_delta[indices]])
    pivot = np.clip(-ref, starts, ends)
    candidate = np.clip(np.divide(-g, h, out=pivot, where=h > 0), starts, ends)
    value = constant + candidate*(2*g + candidate*h)
    error = 128*np.finfo(np.longdouble).eps*max(1., len(a), np.max(np.abs(value)))
    better = np.flatnonzero(value <= value.min()+error)
    offsets = np.unique(candidate[better]+ref)
    exact = np.array([np.minimum((a+s*b)**2, tau2).sum() for s in offsets])
    # Prefer proximity to zero for numerically flat intervals.
    tied = np.flatnonzero(exact <= exact.min()+error)
    best = tied[np.argmin(abs(offsets[tied]))]
    return float(offsets[best]), float(exact[best])


def cloud_key(points):
    xyz = np.ascontiguousarray(points, dtype="<f4")
    return hashlib.sha256(memoryview(xyz).cast("B")).hexdigest()


def build_cache(points, path, k=20, batch=65536):
    """Persist the Q9 PCA normals and widths in target point order."""
    path = Path(path)
    xyz = np.ascontiguousarray(points, dtype="<f4")
    if len(xyz) < k or k < 3:
        raise ValueError("insufficient cloud support for normals")
    tree = cKDTree(xyz)
    normal = np.empty((len(xyz), 3), dtype=np.float64)
    extents = np.empty_like(normal)
    for start in range(0, len(xyz), batch):
        end = min(start+batch, len(xyz))
        indices = tree.query(xyz[start:end], k=k)[1]
        centered = xyz[indices].astype(np.float64)
        centered -= centered.mean(axis=1,keepdims=True)
        eigen, basis = np.linalg.eigh(centered.transpose(0,2,1) @ centered / (k-1))
        normal[start:end] = basis[:,:,0]
        extents[start:end] = np.sqrt(np.maximum(eigen,0))
    fd, temporary = tempfile.mkstemp(dir=path.parent, suffix=".npz")
    try:
        with os.fdopen(fd, "wb") as output:
            np.savez(output, xyz=xyz, normal=normal, extents=extents, k=k, key=cloud_key(xyz))
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


class PatchMap:
    """Load persistent arrays once and reuse a resident spatial index per map."""
    def __init__(self, path, expected_key):
        with np.load(path, allow_pickle=False) as data:
            self.xyz = data["xyz"]
            if cloud_key(self.xyz) != expected_key or str(data["key"]) != expected_key:
                raise ValueError("normal cache target identity mismatch")
            self.normal = data["normal"]
            self.extents = data["extents"]
        self.tree = cKDTree(self.xyz)
