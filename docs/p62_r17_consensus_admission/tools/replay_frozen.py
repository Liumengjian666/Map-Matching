#!/usr/bin/env python3
"""Static no-reference decision replay on R16's frozen common-TP records."""
import csv
import math
import sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


def read(path):
    with open(path, newline='') as stream:
        return list(csv.DictReader(stream))


def pose(row):
    return np.array([float(row[f'pose{i}{j}']) for i in range(4) for j in range(4)]).reshape(4, 4)


def consistent(a, b):
    pa, pb = pose(a), pose(b)
    dt = float(np.linalg.norm(pa[:3, 3] - pb[:3, 3]))
    dr = math.degrees(Rotation.from_matrix(pa[:3, :3].T @ pb[:3, :3]).magnitude())
    return dt, dr


def quality(r):
    return (r['finite'] == '1' and r['converged'] == '1'
            and r['coverage_valid'] == '1' and float(r['overlap_0p5']) >= .5)


def main():
    source = Path(sys.argv[1]); target = Path(sys.argv[2])
    rows = read(source)
    assert len(rows) == 60
    by = {(int(r['frame']), r['method']): r for r in rows}
    with target.open('x', newline='') as stream:
        out = csv.writer(stream)
        out.writerow(['frame', 'raw_ordinal', 'full_rotation_TP_deg', 'full_translation_TP_m',
                      'full_split_translation_m', 'full_split_rotation_deg', 'baseline_accept',
                      'relaxed_accept', 'consensus_accept', 'consensus_extra_split_would_be_needed',
                      'replay_type'])
        for frame in range(1, 31):
            f = by[frame, 'CONTROL_FULL']; s = by[frame, 'CONTROL_SPLIT']
            dr = float(f['rotation_from_TP_deg']); dt = float(f['translation_from_TP_m'])
            dts, drs = consistent(f, s)
            valid = quality(f) and dt <= .8
            event = valid and 5 < dr <= 6
            baseline = valid and dr <= 5
            relaxed = valid and dr <= 6
            consensus = baseline or (event and quality(s) and dts <= .1 and drs <= 1)
            out.writerow([frame, f['raw_ordinal'], dr, dt, dts, drs, int(baseline),
                          int(relaxed), int(consensus), int(event), 'STATIC_COMMON_TP_NOT_CAUSAL_TRAJECTORY'])


if __name__ == '__main__':
    main()
