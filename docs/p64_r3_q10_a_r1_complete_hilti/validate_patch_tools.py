"""Frozen IILABS correspondences for solver/cache engineering checks."""
import argparse
from pathlib import Path
import sys
from time import perf_counter

import numpy as np

from patch_tools import PatchMap, build_cache, cloud_key, least_squares, trunc_squared

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"p64_r3_q9_public_benchmark_fine_geometry"))
from paired import association, normals, read_pcd, transform, write

GRID = np.linspace(-.30,.30,61)


def independent(a, b, tau):
    """Direct interval oracle, independently rebuilding every interval's active set."""
    moving = b != 0
    crossing = np.concatenate(((-tau-a[moving])/b[moving], (tau-a[moving])/b[moving]))
    edges = np.unique(np.r_[-.3, crossing[(crossing>-.3) & (crossing<.3)], .3])
    candidates = list(edges)
    for lo, hi in zip(edges[:-1], edges[1:]):
        active = np.abs(a+b*(lo+hi)/2)<tau
        candidates.append(least_squares(a[active], b[active], lo, hi))
    return min(np.minimum((a+s*b)**2,tau**2).sum() for s in candidates)


def validate(root, out, work):
    rng = np.random.default_rng(91)
    for _ in range(200):
        a,b = rng.normal(size=(2,30))
        _, computed = trunc_squared(a,b,-.3,.3,.1)
        assert np.isclose(computed,independent(a,b,.1),rtol=1e-12,atol=1e-12)
    assert trunc_squared(np.array([1.]),np.array([0.]),-.3,.3,.1)[0]==0
    assert trunc_squared(np.array([.1]),np.array([1.]),-.3,.3,.1)[0]==-.1
    result, time_rows = [], []
    for case in ("W1","W2","W3"):
        folder = root/case
        xyz, source = (read_pcd(folder/file) for file in ("dense_target.pcd","source.pcd"))
        seed = np.loadtxt(folder/"seed.tsv")
        pose, axis = seed[:16].reshape(4,4),seed[16:]
        s = perf_counter()
        plain = normals(xyz.astype(float))
        cold = perf_counter()-s
        s = perf_counter()
        cache = work/f"{case}_normals.npz"
        build_cache(xyz,cache)
        offline = perf_counter()-s
        s = perf_counter()
        index = PatchMap(cache,cloud_key(xyz))
        opened = perf_counter()-s
        query = lambda g: association(g[0],xyz,source,pose,g[1],g[2],axis)
        t = perf_counter()
        first = query((index.tree,index.normal,index.extents))
        first_s = perf_counter()-t
        hot=[]
        for _ in range(10):
            t = perf_counter()
            repeated = query((index.tree,index.normal,index.extents))
            hot.append(perf_counter()-t)
        legacy = query(plain)
        assert all(np.array_equal(x,y) for x,y in zip(first[:3],legacy[:3]))
        assert all(np.array_equal(x,y) for x,y in zip(first,repeated))
        time_rows.append(dict(case=case,status="IILABS_DEVELOPMENT",points=len(xyz),bytes=cache.stat().st_size,
            uncached_normals_s=cold,offline_s=offline,online_load_and_tree_s=opened,first_query_s=first_s,
            hot_query_median_s=np.median(hot),hot_query_p95_s=np.percentile(hot,95),matching_masks_identical=True))
        ids,full,fine,_ = first
        for key, mask in (("full",full),("fine",fine)):
            normal = index.normal[ids[mask]]
            a = np.einsum("ij,ij->i", transform(source[mask],pose)-xyz[ids[mask]], normal)
            b = normal @ axis
            cost = lambda s: np.minimum((a+s*b)**2,.01).sum()
            grid = GRID[np.argmin([cost(x) for x in GRID])]
            exact,val = trunc_squared(a,b,-.3,.3,.1)
            ols = least_squares(a,b,-.3,.3)
            assert val <= cost(grid)+1e-10
            assert np.isclose(val,independent(a,b,.1),rtol=1e-12,atol=1e-10)
            result.append(dict(case=case,pool=key,status="IILABS_FIXED_NUMERICAL_CHECK",points=len(a),
                grid_m=grid,grid_truncated_mse=cost(grid)/len(a),exact_m=exact,exact_truncated_mse=val/len(a),
                ols_m=ols,ols_squared_mse=np.mean((a+ols*b)**2),ols_truncated_mse=cost(ols)/len(a)))
    write(out/"NUMERICAL_SOLVER_COMPARISON.csv",result)
    write(out/"RUNTIME_AND_CACHE_ANALYSIS.csv",time_rows)


if __name__=="__main__":
    p = argparse.ArgumentParser()
    for name in ("root","out","work"):
        p.add_argument(name,type=Path)
    a=p.parse_args()
    validate(a.root,a.out,a.work)
