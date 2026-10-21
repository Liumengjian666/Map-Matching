"""Bounded, offline asset inspection. No registration, fitting, or frame selection.

Writes exclusively to this new archive using exclusive creation. Historical
manifests supply expected hashes, not publisher authentication or experiment
authorization. Sampled PCD checks do not certify all scan contents.
"""
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import re
import stat
import struct
import subprocess

HERE = Path(__file__).resolve().parent
DATA = Path('/home/jian/livox_ws/p59_r3_newer_college_official_internal_recovery_data')
SEQ = DATA / '01_short_experiment'
STABLE = Path('/home/jian/livox_ws/dog_visual_loc_ws')
BASE = 'a611f385025788481a747b3649ea4a90f3316540'


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def save(name, value):
    with (HERE / name).open('x') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')


def git(*args):
    env = dict(os.environ, GIT_OPTIONAL_LOCKS='0')
    return subprocess.check_output(['git', '-C', str(STABLE), *args], env=env)


def stable_snapshot():
    """Content snapshot, without source imports or index-refreshing writes."""
    names = set(git('ls-files', '-z').split(b'\0'))
    names.update(git('ls-files', '--others', '--exclude-standard', '-z').split(b'\0'))
    files = {}
    for raw in sorted(names - {b''}):
        name = os.fsdecode(raw)
        path = STABLE / name
        if not path.exists() and not path.is_symlink():
            files[name] = {'status': 'MISSING'}
            continue
        info = path.lstat()
        if stat.S_ISLNK(info.st_mode):
            files[name] = {'symlink': os.readlink(path)}
        elif stat.S_ISREG(info.st_mode):
            files[name] = {'size': info.st_size, 'sha256': sha(path)}
        else:
            raise ValueError(f'Unsupported protected entry: {path}')
    index = Path(os.fsdecode(git('rev-parse', '--git-path', 'index')).strip())
    if not index.is_absolute():
        index = STABLE / index
    return {
        'head': git('rev-parse', 'HEAD').decode().strip(),
        'branch': git('symbolic-ref', '--short', 'HEAD').decode().strip(),
        'index_sha256': sha(index),
        'refs': git('show-ref').decode(),
        'reflog': git('reflog', '--all', '--format=%H %gD %gs').decode(),
        'status': git('status', '--porcelain=v1', '-z', '--untracked-files=all').decode(),
        'files': files,
        'scope': 'tracked and nonignored untracked files; ignored build files excluded',
    }


def pcd_sample(path):
    with path.open('rb') as stream:
        header = {}
        for _ in range(64):
            line = stream.readline(4097)
            if len(line) > 4096 or not line:
                raise ValueError('Malformed PCD header')
            text = line.decode('ascii').strip()
            if not text or text.startswith('#'):
                continue
            key, *values = text.split()
            header[key] = values
            if key == 'DATA':
                break
        if header.get('DATA') != ['binary']:
            raise ValueError('Only sampled uncompressed binary PCD supported')
        fields = header['FIELDS']
        sizes = list(map(int, header['SIZE']))
        types = header['TYPE']
        counts = list(map(int, header.get('COUNT', ['1'] * len(fields))))
        if not (len(fields) == len(sizes) == len(types) == len(counts)):
            raise ValueError('PCD field schema mismatch')
        offsets = []
        step = 0
        for width, count in zip(sizes, counts):
            offsets.append(step)
            step += width * count
        count = int(header['POINTS'][0])
        payload = stream.read()
        if len(payload) != step * count:
            return {'path': str(path), 'size': path.stat().st_size, 'sha256': sha(path),
                    'header': header, 'point_step': step,
                    'declared_payload_bytes': step * count, 'actual_payload_bytes': len(payload),
                    'trailing_bytes': max(0, len(payload) - step * count),
                    'trailing_all_zero': not any(payload[step * count:]),
                    'decode_status': 'NONCANONICAL_LENGTH_NEEDS_INDEPENDENT_PCL_READ',
                    'finite_xyz_count': None, 't_min': None, 't_max': None}
        t_index = fields.index('t')
        if (sizes[t_index], types[t_index], counts[t_index]) != (4, 'U', 1):
            raise ValueError('Unexpected point time storage')
        times = [struct.unpack_from('<I', payload, i * step + offsets[t_index])[0]
                 for i in range(count)]
        finite_xyz = 0
        for i in range(count):
            xyz = []
            for field in ('x', 'y', 'z'):
                index = fields.index(field)
                if (sizes[index], types[index], counts[index]) != (4, 'F', 1):
                    raise ValueError('Unexpected XYZ storage')
                xyz.append(struct.unpack_from('<f', payload, i * step + offsets[index])[0])
            finite_xyz += all(map(math.isfinite, xyz))
    return {'path': str(path), 'size': path.stat().st_size, 'sha256': sha(path),
            'header': header, 'point_step': step, 'finite_xyz_count': finite_xyz,
            't_min': min(times), 't_max': max(times),
            'limit': 'only this sampled scan decoded; t unit needs source evidence'}


def main():
    if (HERE / 'ASSET_MANIFEST.json').exists():
        raise SystemExit('Refusing to overwrite previous inspection')
    before = stable_snapshot()
    save('PRODUCTION_BEFORE.json', before)
    expected = json.loads((DATA / 'download_status.json').read_text())['files']
    paths = [SEQ / 'ground_truth/registered_poses.csv', SEQ / 'prior_map/README.txt',
             SEQ / 'prior_map/new-college-29-01-2020-5cm-resolution.ply',
             SEQ / 'prior_map/prior_map_xyz_binary.pcd',
             SEQ / 'time_offsets/time_offsets.csv']
    paths.extend(sorted((SEQ / 'calibration').rglob('*')))
    paths = [p for p in paths if p.is_file()]
    assets = []
    for path in paths:
        rel = str(path.relative_to(DATA))
        entry = {'path': str(path), 'size': path.stat().st_size, 'sha256': sha(path)}
        historical = expected.get(rel)
        entry['historical_expected'] = historical
        entry['matches_historical_manifest'] = (
            entry['size'] == historical['bytes'] and entry['sha256'] == historical['sha256']
        ) if historical else None
        if historical and not entry['matches_historical_manifest']:
            raise ValueError(f'Identity mismatch: {path}')
        assets.append(entry)
    scans = []
    for path in (SEQ / 'raw_format/ouster_scan').iterdir():
        match = re.fullmatch(r'cloud_(\d+)_(\d{1,9})\.pcd', path.name)
        if match:
            sec, nsec = map(int, match.groups())
            if not 0 <= nsec < 10**9 or not path.is_file() or path.is_symlink():
                raise ValueError(f'Illegal scan identity: {path}')
            scans.append((sec * 10**9 + nsec, path))
    scans.sort()
    if not scans or any(b[0] <= a[0] for a, b in zip(scans, scans[1:])):
        raise ValueError('No scans or duplicate/nonincreasing scan timestamps')
    with (HERE / 'LOCAL_SCAN_INVENTORY.csv').open('x', newline='') as stream:
        out = csv.writer(stream)
        out.writerow(['inventory_index_zero_based', 'filename_stamp_ns', 'path', 'size_bytes'])
        out.writerows((i, ns, str(p), p.stat().st_size) for i, (ns, p) in enumerate(scans))
    rows = []
    with (SEQ / 'ground_truth/registered_poses.csv').open() as stream:
        reader = csv.reader(stream)
        if next(reader) != ['#sec', 'nsec', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']:
            raise ValueError('Reference schema mismatch')
        norm_errors = []
        for row in reader:
            if len(row) != 9:
                raise ValueError('Reference row length')
            sec, nsec = map(int, row[:2])
            values = list(map(float, row[2:]))
            if not 0 <= nsec < 10**9 or not all(map(math.isfinite, values)):
                raise ValueError('Nonfinite reference or invalid nanoseconds')
            norm_errors.append(abs(math.sqrt(sum(x*x for x in values[3:])) - 1))
            rows.append(sec * 10**9 + nsec)
    if not rows or any(b <= a for a, b in zip(rows, rows[1:])):
        raise ValueError('Reference timestamps not strictly increasing')
    row_set = set(rows)
    samples = [pcd_sample(scans[i][1]) for i in (0, len(scans)//2, len(scans)-1)]
    manifest = {'baseline': BASE, 'dataset': 'Newer College 2020 / 01_short_experiment',
                'historical_manifest_path': str(DATA / 'download_status.json'),
                'historical_manifest_sha256': sha(DATA / 'download_status.json'),
                'assets': assets, 'sampled_pcd': samples,
                'publisher_hash_authentication': 'NOT_AVAILABLE; historical LOCAL hashes only',
                'pcd_full_content_identity': 'NOT_VERIFIED_FOR_ALL_SCANS',
                'lidar_scan_count': len(scans),
                'lidar_file_bytes': sum(p.stat().st_size for _, p in scans),
                'lidar_filename_time_start_ns': scans[0][0],
                'lidar_filename_time_end_ns': scans[-1][0],
                'reference_row_count': len(rows),
                'reference_time_start_ns': rows[0], 'reference_time_end_ns': rows[-1],
                'reference_quaternion_norm_max_error': max(norm_errors),
                'exact_scan_reference_timestamp_pairs': sum(ns in row_set for ns, _ in scans),
                'time_semantic_closure': 'NOT_CERTIFIED_BY_INTEGER_EQUALITY',
                'test_frames_selected': False, 'ndt_align': 0, 'pose_errors_computed': False,
                'reference_poses_read_for_format_audit': True}
    # Verify content identities again after inspection; no automatic repair.
    for entry in assets + samples:
        path = Path(entry['path'])
        if path.stat().st_size != entry['size'] or sha(path) != entry['sha256']:
            raise ValueError(f'Asset changed during inspection: {path}')
    after = stable_snapshot()
    save('PRODUCTION_AFTER.json', after)
    if before != after:
        raise ValueError('Protected production state changed; stop')
    manifest['production_snapshot_equal'] = True
    manifest['inspected_asset_before_after_equal'] = True
    save('ASSET_MANIFEST.json', manifest)
    print(json.dumps({k: v for k, v in manifest.items()
                      if k not in ('assets', 'sampled_pcd')}, indent=2))


if __name__ == '__main__':
    main()
