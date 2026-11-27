"""Build the nominal explanation-state matrix from map directions and fixed point IDs."""
from pathlib import Path
import csv
import sys
import time
from collections import Counter
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import cKDTree

from freeze import PAIRS, digest, read, write

def explain(source, target, pose, maximum):
    target = (target-pose[:3,3]) @ pose[:3,:3]
    r = np.linalg.norm(source,axis=1)
    expected = np.linalg.norm(target,axis=1)
    valid = np.isfinite(expected) & (expected>=.5) & (expected<=maximum)
    expected, target = expected[valid], target[valid]
    x = source / np.maximum(r[:,None], .001)
    dr, ids = cKDTree(target/expected[:,None]).query(x,k=3)
    distances = expected[ids]
    support = (r>=.5) & (r<=maximum) & (dr[:,-1]<=2*np.sin(np.deg2rad(1.5)/2))
    support &= np.ptp(distances,axis=1)<=.15
    depth = r-np.median(distances,axis=1)
    state = np.zeros(len(source),np.uint8)
    state[support & (np.abs(depth)<=.15)] = 1
    state[support & (depth>.25)] = 2
    confidence = np.where(state>0,np.exp(-np.abs(depth)/.15),np.nan)
    return state,confidence,depth

def entropy(state):
    count = (state>0).sum(axis=0)
    ratio = np.divide((state==1).sum(axis=0),count,out=np.full(count.shape,np.nan),where=count>0)
    middle = (ratio>0)&(ratio<1)
    h = np.zeros(len(count))
    r = ratio[middle]
    h[middle] = -r*np.log2(r)-(1-r)*np.log2(1-r)
    h[count<2] = np.nan
    return ratio,h

def nominal(run, out):
    rows, timings = [], []
    fig, axes = plt.subplots(1,4,figsize=(13,3.7),layout='constrained')
    for ax, sample in zip(axes,read(out/'sample_manifest.csv')):
        tick = time.perf_counter()
        frame = sample['frame']
        base = run/f'{frame}_nominal'
        assert digest(base/'source.bin') == sample['q1_filtered_source_sha256']
        assert digest(base/'target.bin') == sample['q1_filtered_target_sha256']
        source = np.fromfile(base/'source.bin','<f4').reshape(-1,3)
        target = np.fromfile(base/'map.bin','<f4').reshape(-1,3)
        poses = np.load(run/f'{frame}_poses.npz')['nominal']
        state, confidence, depth = map(np.stack,zip(*[
            explain(source,target,pose,12 if frame.startswith('O') else 8) for pose in poses]))
        ratio,h = entropy(state)
        scores = np.stack([np.fromfile(base/f'{k}_0.bin','<f8').reshape(-1,2) for k in range(len(poses))])
        if frame.startswith(('H307','H515')):
            pair = {'H307':'H307_correct_wrong','H515':'H515_boundary_rank'}[frame]
            ref = Path(__file__).resolve().parent.parent/'p64_r2_q2_bounded_search_adaptation/r3_fixed_assets'
            sealed = np.genfromtxt(ref/f'{pair}_point_deltas.csv.gz',names=True,delimiter=',')
            np.testing.assert_array_equal(sealed['s_a'],scores[0,:,0])
            np.testing.assert_array_equal(sealed['s_b'],scores[1,:,0])
        np.savez_compressed(run/f'observation_{frame}.npz',state=state,confidence=confidence,depth=depth,
                            match_ratio=ratio,entropy=h,nominal_scores=scores[:,:,0])
        keys = ['UNKNOWN','MATCHED','INCONSISTENT']
        for k in range(len(poses)):
            for s,key in enumerate(keys):
                n = (state[k]==s).sum()
                rows.append(dict(frame=frame,pair='',candidate=k,class_name=key,n=n,fraction=n/len(source),mean_abs_delta=''))
        for name,a,b in PAIRS[frame]:
            delta = np.abs(scores[a,:,0]-scores[b,:,0])
            for (x,y) in [(0,0),(0,1),(0,2),(1,0),(1,1),(1,2),(2,0),(2,1),(2,2)]:
                mask = (state[a]==x)&(state[b]==y)
                rows.append(dict(frame=frame,pair=name,candidate='',class_name=f'{keys[x]}/{keys[y]}',n=mask.sum(),
                                 fraction=mask.mean(),mean_abs_delta=delta[mask].mean() if mask.any() else ''))
        group = np.where((state[:2]>0).all(axis=0),np.where(state[0]==state[1],1,2),0)
        for value,color,name in [(0,'gray','unknown'),(1,'#23876d','same'),(2,'#d34a33','disagreement')]:
            mask = group==value
            ax.scatter(source[mask,0],source[mask,2],s=2,c=color,label=name,rasterized=True)
        ax.set_title(frame)
        ax.set_aspect('equal')
        ax.set_xlabel('x sensor (m)')
        timings.append(dict(frame=frame,stage='observation_proxy_and_asset',wall_s=time.perf_counter()-tick))
    axes[0].set_ylabel('z sensor (m)')
    axes[-1].legend(fontsize=8)
    fig.savefig(out/'observation_matrix_spatial.png',dpi=165)
    write(out/'observation_matrix_results.csv',rows)
    write(out/'runtime_observation.csv',timings)
if __name__ == '__main__':
    nominal(Path(sys.argv[1]),Path(sys.argv[2]))
