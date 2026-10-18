"""Preregistered descriptive labels; these are not runtime detector thresholds."""
import math
import numpy as np

ARMS = ('FULL_360', 'FOV_90', 'POINT_COUNT_MATCHED')


def parse_resource(text):
    result = {}
    for line in text.splitlines():
        if 'User time (seconds):' in line:
            result['user_sec'] = float(line.rsplit(':', 1)[1])
        elif 'System time (seconds):' in line:
            result['system_sec'] = float(line.rsplit(':', 1)[1])
        elif 'Maximum resident set size (kbytes):' in line:
            result['max_rss_kib'] = int(line.rsplit(':', 1)[1])
        elif 'Elapsed (wall clock) time' in line:
            value = 0.
            for field in line.rsplit('): ', 1)[1].strip().split(':'):
                value = value * 60 + float(field)
            result['wall_sec'] = value
    if len(result) != 4:
        raise ValueError('incomplete process resource log')
    return result


def weak_label(row):
    return (row['spd'] == '1' and float(row['ratio6']) <= 1e-3
            and float(row['ratio_translation']) <= 1e-2)


def axis_similarity(a, b, prefix='translation_weak', dimensions=3):
    x = np.array([float(a[prefix + str(i)]) for i in range(dimensions)])
    y = np.array([float(b[prefix + str(i)]) for i in range(dimensions)])
    denominator = np.linalg.norm(x) * np.linalg.norm(y)
    return float(abs(x @ y) / denominator) if denominator > 0 else float('nan')


def classify(primary):
    ids = sorted(primary[ARMS[0]])
    comparable = [k for k in ids if all(primary[a][k]['comparable'] == '1' for a in ARMS)]
    spd = [k for k in comparable if all(primary[a][k]['spd'] == '1' for a in ARMS)]
    native_runs, current = [], []
    for k in ids:
        row = primary[ARMS[0]][k]
        if row['comparable'] == '1' and weak_label(row):
            if current and axis_similarity(primary[ARMS[0]][current[-1]], row) < math.cos(math.radians(20)):
                if len(current) >= 3:
                    native_runs.append(current)
                current = []
            current.append(k)
        else:
            if len(current) >= 3:
                native_runs.append(current)
            current = []
    if len(current) >= 3:
        native_runs.append(current)
    ratios = [float(primary[ARMS[1]][k]['ratio6']) / float(primary[ARMS[2]][k]['ratio6']) for k in spd]
    fraction = sum(r < 1 for r in ratios) / len(ratios) if ratios else 0.
    only_fov = sum(weak_label(primary[ARMS[1]][k]) and not weak_label(primary[ARMS[2]][k]) for k in spd)
    median = float(np.median(ratios)) if ratios else None
    result = 'DEGENERACY_NOT_DEMONSTRATED'
    if len(comparable) < 80:
        result = 'GEOMETRY_SCREEN_INSUFFICIENT'
    elif native_runs:
        result = 'NATIVE_DEGENERACY_EVIDENCE_OBSERVED'
    elif len(spd) >= 80 and median <= .5 and fraction >= .75 and only_fov >= 20:
        result = 'FOV_INDUCED_DEGENERACY_OBSERVED'
    return {'final_result': result, 'comparable_triplets': len(comparable),
            'spd_comparable_triplets': len(spd), 'native_sampled_candidate_runs': native_runs,
            'paired_fov_count_ratio6_median': median, 'fraction_fov_ratio6_lower_than_count': fraction,
            'only_fov_weak_label_pairs': int(only_fov)}


def distribution(values):
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    return {'n': len(values), 'min': float(np.min(values)) if len(values) else None,
            'mean': float(np.mean(values)) if len(values) else None,
            'median': float(np.median(values)) if len(values) else None,
            'p95': float(np.percentile(values, 95)) if len(values) else None,
            'max': float(np.max(values)) if len(values) else None}
