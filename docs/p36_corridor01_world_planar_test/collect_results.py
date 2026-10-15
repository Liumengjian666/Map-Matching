#!/usr/bin/env python3
"""Verify saved probe output, import P33 controls, plot; never query NN."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import re
import shutil

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d

HERE = Path(__file__).resolve().parent
WS = HERE.parents[1]
P33 = WS/'docs/p33_corridor01_complete_frame_relation'
LABELS = ['C0_GT', 'C1_GT_E', 'A_Y_GT', 'B_INVERSE_Y_GT']


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save(name, obj):
    with (HERE/name).open('x') as stream:
        json.dump(obj, stream, indent=2, allow_nan=False); stream.write('\n')


def csv_write(name, rows):
    with (HERE/name).open('x', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator='\n')
        writer.writeheader(); writer.writerows(rows)


def pose(row):
    return np.array([float(row[f'T{r}{c}']) for r in range(4) for c in range(4)]).reshape(4, 4)


def main(cache):
    frozen = json.loads((cache/'input/input_freeze.json').read_text())
    if sha(P33/'direct_projection.csv') != frozen['P33_controls_SHA256']:
        raise ValueError('CONTROLS_CHANGED')
    if sha(P33/'input_freeze.json') != frozen['P33_freeze_SHA256']:
        raise ValueError('P33_FREEZE_CHANGED')
    if sha(HERE/'PROTOCOL.md') != frozen['protocol_SHA256']:
        raise ValueError('PROTOCOL_CHANGED')
    with (P33/'direct_projection.csv').open() as stream:
        controls = list(csv.DictReader(stream))
    if len(controls) != 6 or len(frozen['samples']) != 3:
        raise ValueError('FRAME_BUDGET')
    original_map = np.asarray(o3d.io.read_point_cloud(frozen['target_map']).points)
    map_plot = original_map[::4]  # Display only; all NN uses the frozen PCL target.
    fig, axes = plt.subplots(3, 2, figsize=(14, 14))
    rows, resources, parity = [], [], []
    for i, sample in enumerate(frozen['samples']):
        root = cache/'input'/str(sample['target_s'])/'geometry'
        old = [r for r in controls if r['scan_ns'] == sample['scan_ns']]
        with (root/'geometry.csv').open() as stream:
            new = list(csv.DictReader(stream))
        if len(old) != 2 or len(new) != 2 or (root/'ndt.csv').exists():
            raise ValueError('NN_ONLY_BUDGET')
        if [r['hypothesis'] for r in new] != ['H1_GT_LIDAR_W_EQ_M', 'H2_GT_IMU_W_EQ_M']:
            raise ValueError('PROBE_SLOT_ORDER')
        actual_sha = sha(root/'preprocessed_source.xyz')
        if (actual_sha != sample['P33_preprocessed_XYZ_SHA256'] or
                (root/'preprocessed_source.xyz').read_bytes() != Path(sample['P33_preprocessed_source']).read_bytes()):
            raise ValueError('P33_ORDERED_PREPROCESS_PARITY')
        stdout = (root/'stdout.txt').read_text()
        counts = re.findall(r'NN_queries=(\d+) full_NDT_aligns=(\d+).*source_stable_hash=([0-9a-f]+)', stdout)
        if len(counts) != 1 or counts[0][:2] != ('2', '0'):
            raise ValueError('ACTUAL_CALL_COUNTS')
        points = np.fromfile(root/'preprocessed_source.xyz', dtype='<f4').reshape(-1, 3)
        expected_poses = [sample[k] for k in ['T_G_GT', 'H_B_conditional', 'A_Y_GT', 'B_INVERSE_Y_GT']]
        for j, (label, item, expected) in enumerate(zip(LABELS, old+new, expected_poses)):
            T = pose(item)
            if not np.allclose(T, expected, rtol=0, atol=1e-14):
                raise ValueError('SAVED_MATRIX_CHANGED')
            if int(item['source_points']) != 1400 or int(item['target_points']) != 225826:
                raise ValueError('POINTSET_SIZE_CHANGED')
            for k in ['nn_median_m', 'nn_p90_m', 'coverage_0_5m', 'coverage_2m', 'coverage_8m']:
                if not np.isfinite(float(item[k])):
                    raise ValueError('NONFINITE_METRIC')
            rows.append(dict(target_s=sample['target_s'], raw_frame_index=sample['raw_frame_index'],
                scan_ns=sample['scan_ns'], group=label,
                evidence='P33 frozen control' if j < 2 else 'P36 fixed NN query',
                **{k:v for k,v in item.items() if k not in ['hypothesis', 'target_s', 'raw_frame_index', 'scan_ns']}))
            projected = points@T[:3, :3].astype('<f4').T+T[:3, 3].astype('<f4')
            for col, (d0, d1) in enumerate([(0, 1), (0, 2)]):
                ax = axes[i, col]
                if j == 0:
                    ax.scatter(map_plot[:, d0], map_plot[:, d1], s=.3, c='.7', label='Original PCD (display subset)')
                ax.scatter(projected[:, d0], projected[:, d1], s=2, alpha=.65,
                           c=['tab:blue', 'tab:orange', 'tab:green', 'tab:red'][j], label=label)
                ax.set_xlabel('XYZ'[d0]+' (m)'); ax.set_ylabel('XYZ'[d1]+' (m)')
                ax.set_title(f'raw{sample["raw_frame_index"]}, header +{sample["relative_header_s"]}s')
                ax.axis('equal'); ax.grid(alpha=.2)
        parity.append(dict(raw_frame_index=sample['raw_frame_index'], scan_ns=sample['scan_ns'],
            raw_XYZ_SHA256=sample['raw_XYZ_SHA256'], preprocessed_XYZ_SHA256=actual_sha,
            stableCloudHash=counts[0][2], preprocessed_bytes_equal_P33=True,
            source_points=len(points), target_points=225826, GT_interpolation_equal_P33=True))
        text = (root/'resources.txt').read_text()
        def value(prefix):
            return next(line.strip().rsplit(': ', 1)[1] for line in text.splitlines() if line.strip().startswith(prefix))
        wall = value('Elapsed (wall clock)'); minute, second = wall.split(':')
        resources.append(dict(raw_frame_index=sample['raw_frame_index'], NN_queries=2, NDT_aligns=0, ICP_calls=0,
            wall_s=60*int(minute)+float(second), CPU_user_s=float(value('User time')),
            CPU_system_s=float(value('System time')), peak_RSS_KiB=int(value('Maximum resident set size'))))
    for ax in axes[:, 0]:
        ax.legend(markerscale=3, fontsize=7)
    fig.suptitle('P36: fixed World/Planar direction hypotheses, original map; no optimization')
    fig.tight_layout(rect=[0, 0, 1, .98]); fig.savefig(HERE/'xy_xz_projection.png', dpi=130); plt.close(fig)
    csv_write('nn_comparison.csv', rows); csv_write('source_parity.csv', parity)
    csv_write('runtime_summary.csv', resources)
    shutil.copyfile(cache/'input/input_freeze.json', HERE/'input_freeze.json')
    save('projection_evidence.json', dict(rows=rows, resources=resources, parity=parity,
        calls=dict(new_NN_statistics=6, inherited_P33_statistics=6, ICP=0, NDT=0, localization_replays=0),
        map_bounds=dict(min=original_map.min(0).tolist(), max=original_map.max(0).tolist())))
    print(json.dumps([dict(frame=r['raw_frame_index'], group=r['group'],
        median=float(r['nn_median_m']), P90=float(r['nn_p90_m']),
        overlap05=float(r['coverage_0_5m']), overlap2=float(r['coverage_2m']), overlap8=float(r['coverage_8m']))
        for r in rows], indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cache', type=Path, required=True)
    main(**vars(parser.parse_args()))
