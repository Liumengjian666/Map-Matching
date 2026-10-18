"""GT-only feasibility diagnostics. No poses generated for mapping/localization."""
import csv
from decimal import Decimal, ROUND_HALF_EVEN
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
from scipy.spatial import cKDTree
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

DOC = Path(__file__).resolve().parent
R2 = DOC.parent / 'p61_r2_geode_real_data_run'


def sha(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def ns(text):
    return int((Decimal(text) * 1000000000).to_integral_value(rounding=ROUND_HALF_EVEN))


def read_gt(path):
    lines = [line.split() for line in Path(path).read_text().splitlines() if line.strip() and not line.lstrip().startswith('#')]
    if not lines or any(len(row) != 8 for row in lines): raise ValueError('GT not 8-column numeric text')
    stamps = np.array([ns(row[0]) for row in lines], dtype=np.int64)
    data = np.array([[float(x) for x in row[1:]] for row in lines])
    if not np.isfinite(data).all() or np.any(np.diff(stamps) <= 0):
        raise ValueError('nonfinite or nonmonotonic GT')
    return stamps, data[:, :3], data[:, 3:], [row[0] for row in lines]


def freeze(directory):
    directory = Path(directory); directory.mkdir(parents=True, exist_ok=False)
    with (R2/'scan_ledger.csv').open() as stream: rows = list(csv.DictReader(stream))
    stamps = [int(row['stamp_ns']) for row in rows]
    start, end = stamps[0], stamps[-1]; span = end-start
    result = {'source': 'R2 LiDAR ledger only; no GT coordinates read', 'lidar_frames': len(stamps),
              'lidar_start_ns': start, 'lidar_end_ns': end,
              'mapping_candidate_start_ns': start, 'mapping_candidate_end_ns': start+span*45//100,
              'test_candidate_start_ns': start+span*55//100, 'test_candidate_end_ns': end,
              'ledger_sha256': sha(R2/'scan_ledger.csv'), 'protocol_sha256': sha(DOC/'PROTOCOL.md'),
              'actual_mapping_or_test_executed': False}
    with (directory/'time_split_freeze.json').open('x') as stream: json.dump(result, stream, indent=2)
    print(json.dumps(result, indent=2))


def analyze(gt_file, directory):
    directory = Path(directory); windows = json.loads((directory/'time_split_freeze.json').read_text())
    if windows['protocol_sha256'] != sha(DOC/'PROTOCOL.md'): raise ValueError('preregistered protocol changed')
    t, xyz, q, original_t = read_gt(gt_file)
    m = (t >= windows['mapping_candidate_start_ns']) & (t <= windows['mapping_candidate_end_ns'])
    s = (t >= windows['test_candidate_start_ns']) & (t <= windows['test_candidate_end_ns'])
    with (R2/'scan_ledger.csv').open() as stream: lidar = list(csv.DictReader(stream))
    scans = np.array([int(r['stamp_ns']) for r in lidar], dtype=np.int64)
    norms = np.linalg.norm(q, axis=1)
    gaps = np.diff(t)*1e-9
    result = {'gt_file_sha256': sha(gt_file), 'gt_bytes': Path(gt_file).stat().st_size,
              'rows': len(t), 'columns': 8, 'first_stamp_text': original_t[0], 'last_stamp_text': original_t[-1],
              'first_ns': int(t[0]), 'last_ns': int(t[-1]), 'duration_sec': (int(t[-1])-int(t[0]))*1e-9,
              'dt_median_sec': float(np.median(gaps)), 'dt_max_sec': float(max(gaps)),
              'gaps_over_1_sec': int(np.sum(gaps>1)),
              'quaternion_zero_rows': int(np.sum(norms==0)), 'quaternion_unit_rows': int(np.sum(abs(norms-1)<1e-3)),
              'gt_orientation_usable': bool(np.all(abs(norms-1)<1e-3)),
              'xyz_min': xyz.min(axis=0).tolist(), 'xyz_max': xyz.max(axis=0).tolist(),
              'position_endpoint_separation_m': float(np.linalg.norm(xyz[-1]-xyz[0])),
              'lidar_header_inside_gt_endpoint_range': int(np.sum((scans>=t[0])&(scans<=t[-1]))),
              'mapping_window_gt_rows': int(m.sum()), 'test_window_gt_rows': int(s.sum()),
              'split_chosen_without_gt': True, 'gt_used_for_map_or_initialization': False,
              'overlap_means_reference_position_proximity_only': True}
    # Endpoint inclusion is not evidence of continuous reference support through long GT gaps.
    after = np.searchsorted(t, scans, side='left'); nearest = np.minimum(abs(scans-t[np.clip(after,0,len(t)-1)]),
                                                                     abs(scans-t[np.clip(after-1,0,len(t)-1)]))*1e-9
    result['lidar_headers_within_100ms_of_gt_sample'] = int(np.sum(nearest<=.1))
    if m.sum() and s.sum():
        distances, _ = cKDTree(xyz[m]).query(xyz[s], k=1)
        result['test_to_mapping_reference_distance_m'] = {'min': float(distances.min()),
            'median': float(np.median(distances)), 'p95': float(np.percentile(distances,95)), 'max': float(distances.max())}
        result['test_reference_position_proximity_fraction'] = {str(x): float(np.mean(distances<=x)) for x in (1,2,5)}
    with (directory/'gt_feasibility.json').open('x') as stream: json.dump(result, stream, indent=2, allow_nan=False)
    fig, axes = plt.subplots(1,2,figsize=(12,4))
    axes[0].scatter(xyz[:,0],xyz[:,1],s=3,color='lightgrey',label='Official position reference')
    axes[0].scatter(xyz[m,0],xyz[m,1],s=5,label='First45% candidate (no mapping run)')
    axes[0].scatter(xyz[s,0],xyz[s,1],s=5,label='Last45% candidate (no test run)')
    axes[0].set(xlabel='Official reference X (m)',ylabel='Official reference Y (m)');axes[0].axis('equal');axes[0].legend(fontsize=7)
    for k in range(3): axes[1].scatter((t-t[0])*1e-9,xyz[:,k]-xyz[0,k],s=3,label='XYZ'[k])
    axes[1].set(xlabel='Seconds since first GT record',ylabel='Position relative to first GT (m)');axes[1].legend()
    fig.suptitle('Position-only reference; NOT a LiDAR map or GT-based mapping pose')
    fig.tight_layout();fig.savefig(directory/'reference_split_feasibility.png',dpi=160);plt.close(fig)
    print(json.dumps(result,indent=2))


if __name__=='__main__':
    if sys.argv[1]=='freeze': freeze(sys.argv[2])
    elif sys.argv[1]=='analyze': analyze(sys.argv[2],sys.argv[3])
    else: raise ValueError('freeze|analyze only')
