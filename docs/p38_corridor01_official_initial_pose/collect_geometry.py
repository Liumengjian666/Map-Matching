#!/usr/bin/env python3
"""Collect exactly two saved NN rows; no queries, optimization or GT use."""
import argparse
import csv
import json
from pathlib import Path
import re
import shutil

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
from prepare_initial import sha, ndt_eligible

HERE = Path(__file__).resolve().parent


def save(name, obj):
    with (HERE/name).open('x') as stream:
        json.dump(obj, stream, indent=2, allow_nan=False); stream.write('\n')


def main(cache):
    outputs = ['input_freeze.json', 'gt_diagnostic.json', 'time_binding.csv', 'test_results.txt',
               'geometry_comparison.csv', 'initial_pose_projection.png', 'geometry_gate.json',
               'source_parity.json', 'resource_summary.json', 'probe_stdout.txt']
    if any((HERE/name).exists() for name in outputs):
        raise ValueError('NO_OVERWRITE')
    frozen = json.loads((cache/'input/input_freeze.json').read_text())
    if frozen['raw_frame_index'] != 665 or frozen['scan_ns'] != '1517157286055073023':
        raise ValueError('FRAME_IDENTITY')
    if sha(HERE/'PROTOCOL.md') != frozen['protocol_SHA256']:
        raise ValueError('PROTOCOL_CHANGED')
    root = cache/'geometry'
    with (root/'geometry.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    if (len(rows) != 2 or (root/'ndt.csv').exists() or
            [r['hypothesis'] for r in rows] != ['H1_GT_LIDAR_W_EQ_M', 'H2_GT_IMU_W_EQ_M']):
        raise ValueError('NN_ONLY_SLOT_CONTRACT')
    stdout = (root/'stdout.txt').read_text()
    if len(re.findall(r'NN_queries=2 full_NDT_aligns=0 ', stdout)) != 1:
        raise ValueError('ACTUAL_CALL_COUNTS')
    actual_source = root/'preprocessed_source.xyz'
    if (sha(actual_source) != frozen['preprocessed_XYZ_SHA256'] or
            actual_source.read_bytes() != Path(frozen['P33_preprocessed_source']).read_bytes() or
            actual_source.read_bytes() != Path(frozen['P36_preprocessed_source']).read_bytes()):
        raise ValueError('ORDERED_SOURCE_PARITY')
    raw_map = np.asarray(o3d.io.read_point_cloud(frozen['target_map']).points)
    source = np.fromfile(actual_source, dtype='<f4').reshape(-1, 3)
    projected, output_rows = [], []
    for label, row, key in zip(['A_DIRECT_Y', 'B_INVERSE_Y'], rows, ['Y', 'inverse_Y']):
        T = np.array([float(row[f'T{r}{c}']) for r in range(4) for c in range(4)]).reshape(4, 4)
        if not np.array_equal(T, frozen[key]):
            raise ValueError('INITIAL_MATRIX_CHANGED')
        if int(row['source_points']) != 1400 or int(row['target_points']) != 225826:
            raise ValueError('POINT_COUNT')
        if not all(np.isfinite(float(row[k])) for k in ['nn_median_m', 'nn_p90_m', 'coverage_0_5m', 'coverage_2m', 'coverage_8m']):
            raise ValueError('NONFINITE_METRICS')
        projected.append(source@T[:3, :3].astype('<f4').T+T[:3, 3].astype('<f4'))
        output_rows.append(dict(raw_frame_index=665, scan_ns=frozen['scan_ns'], candidate=label,
            **{k:v for k,v in row.items() if k != 'hypothesis'}, numeric_ndt_gate_pass=ndt_eligible(row)))
    joined = np.vstack([raw_map]+projected)
    if not np.isfinite(joined).all():
        raise ValueError('NONFINITE_PROJECTION')
    lower, upper = joined.min(0), joined.max(0)
    span = float((upper-lower).max()*1.05); center = (upper+lower)/2
    limits = np.column_stack([center-span/2, center+span/2])
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    for ax, (d0, d1) in zip(axes, [(0, 1), (0, 2), (1, 2)]):
        ax.scatter(raw_map[:, d0], raw_map[:, d1], s=.1, c='.7', label='Original official PCD XYZ')
        for j, label in enumerate(['A: Y', 'B: inverse(Y)']):
            ax.scatter(projected[j][:, d0], projected[j][:, d1], s=3, alpha=.7,
                       c=['tab:green', 'tab:red'][j], label=label)
        ax.set_xlabel('XYZ'[d0]+' (m)'); ax.set_ylabel('XYZ'[d1]+' (m)')
        ax.set_xlim(limits[d0]); ax.set_ylim(limits[d1]); ax.set_aspect('equal', adjustable='box')
        ax.grid(alpha=.2); ax.legend(markerscale=3, fontsize=7)
    fig.suptitle('P38 raw665 (+66.966953993s): direct initial poses only, no GT multiplication')
    fig.tight_layout(rect=[0, 0, 1, .96]); fig.savefig(HERE/'initial_pose_projection.png', dpi=140); plt.close(fig)
    with (HERE/'geometry_comparison.csv').open('x', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(output_rows[0]), lineterminator='\n')
        writer.writeheader(); writer.writerows(output_rows)
    eligible = [row['candidate'] for row in output_rows if row['numeric_ndt_gate_pass']]
    save('geometry_gate.json', dict(numeric_eligible_candidates=eligible,
        preferred_candidate_if_visual_structure_valid=eligible[0] if eligible else None,
        criteria='median<=1m AND P90<=3m AND coverage_0.5m>=0.5; visual structure must also be reasonable',
        initial_pose_geometric_support=False if not eligible else None,
        conditional_NDT_budget=1 if eligible else 0, GT_used_for_gate=False,
        plot_axis_limits_m=limits.tolist(), common_axis_span_m=span))
    match = re.search(r'source_stable_hash=([0-9a-f]+)', stdout)
    if not match:
        raise ValueError('SOURCE_STABLE_HASH_MISSING')
    save('source_parity.json', dict(raw_XYZ_SHA256=frozen['raw_XYZ_SHA256'],
        preprocessed_XYZ_SHA256=sha(actual_source), source_stable_hash=match[1],
        ordered_bytes_equal_P33=True, ordered_bytes_equal_P36=True, points=1400, target_points=225826,
        GT_used_for_candidates=False, NN_statistics=2, NDT_align_calls_in_probe=0, ICP_calls=0))
    text = (root/'resources.txt').read_text()
    def value(prefix):
        return next(line.strip().rsplit(': ', 1)[1] for line in text.splitlines() if line.strip().startswith(prefix))
    save('resource_summary.json', dict(scope='One process: map load, source/target preprocessing, two NN statistics',
        wall_time=value('Elapsed (wall clock)'), CPU_user_s=float(value('User time')),
        CPU_system_s=float(value('System time')), peak_RSS_KiB=int(value('Maximum resident set size')),
        preparation_and_plotting_separately_measured=False))
    for name in ['input_freeze.json', 'gt_diagnostic.json', 'time_binding.csv']:
        shutil.copyfile(cache/'input'/name, HERE/name)
    shutil.copyfile(cache/'test_results.txt', HERE/'test_results.txt')
    shutil.copyfile(root/'stdout.txt', HERE/'probe_stdout.txt')
    print(json.dumps(dict(geometry=output_rows, eligible_candidates=eligible), indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cache', type=Path, required=True)
    main(**vars(parser.parse_args()))
