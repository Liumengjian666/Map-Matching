"""Frozen engineering gate only; curvature severity never gates the raw smoke."""
import csv
import json
from pathlib import Path
import sys
import numpy as np


def table(path):
    with path.open() as stream: return list(csv.DictReader(stream))


def check(directory, expected):
    aligns, jets, grid = [table(directory / name) for name in ('alignments.csv', 'hessian.csv', 'grid_audit.csv')]
    if len(aligns) != expected or len(jets) != expected or len(grid) != 3 * expected:
        raise ValueError('missing diagnostic records')
    if len({row['scan_index'] for row in aligns}) != expected: raise ValueError('duplicate scan pairs')
    if any(any(float(row[key]) != float(np.float32(.8)) for key in ('leaf_x', 'leaf_y', 'leaf_z')) for row in grid):
        raise ValueError('actual grid is not .8f')
    if sum(int(row['align_calls']) for row in aligns) != expected: raise ValueError('align budget violated')
    if sum(int(row['jet_calls']) for row in jets) != expected: raise ValueError('jet budget violated')
    result = {'pairs': expected, 'align_calls': expected, 'native_jet_calls': expected, 'grid_audits': len(grid),
              'actual_grid_pass': True, 'converged': sum(r['converged'] == '1' for r in aligns),
              'rigid': sum(r['rigid'] == '1' for r in aligns), 'jet_valid': sum(r['jet_valid'] == '1' for r in jets),
              'comparable': sum(r['comparable'] == '1' for r in jets), 'spd': sum(r['spd'] == '1' for r in jets)}
    result['engineering_smoke_pass'] = all(result[key] == expected for key in ('converged', 'rigid', 'jet_valid', 'comparable'))
    return result


if __name__ == '__main__':
    result = check(Path(sys.argv[1]), int(sys.argv[2]))
    with Path(sys.argv[3]).open('x') as stream: json.dump(result, stream, indent=2)
    print(json.dumps(result, indent=2))
    if int(sys.argv[2]) == 20 and not result['engineering_smoke_pass']: sys.exit(1)
