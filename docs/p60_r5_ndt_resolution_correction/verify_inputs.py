#!/usr/bin/env python3
"""Read-only verification of unchanged internal R4 assets; no extraction/GT."""
import argparse
import csv
import hashlib
import json
from pathlib import Path


def digest(path):
    path = Path(path).resolve()
    if not str(path).startswith('/home/jian/'):
        raise ValueError('only system-internal inputs permitted')
    value = hashlib.sha256()
    with path.open('rb') as stream:
        for data in iter(lambda: stream.read(1 << 20), b''):
            value.update(data)
    return value.hexdigest()


def verify(new_output):
    if new_output.exists():
        raise FileExistsError(new_output)
    r4 = Path(__file__).resolve().parent.parent / 'p60_r4_ground_challenge_ndt_degeneracy'
    archived = json.loads((r4 / 'archive_hashes.json').read_text())
    for name, identity in archived.items():
        path = r4 / name
        if path.stat().st_size != identity['bytes'] or digest(path) != identity['sha256']:
            raise ValueError('R4 archive identity mismatch: ' + name)
    with (r4 / 'sample_manifest.csv').open() as stream:
        samples = list(csv.DictReader(stream))
    if len(samples) != 100:
        raise ValueError('expected frozen 100 pairs')
    paths = {}
    for row in samples:
        for prefix in ['source', 'target']:
            path = row[prefix + '_path']
            actual = digest(path)
            if actual != row[prefix + '_pcd_sha256']:
                raise ValueError('raw PCD changed')
            paths[path] = actual
    bag = Path('/home/jian/ground-challenge/corridor2/corridor2.bag')
    bag_sha = digest(bag)
    if bag_sha != '6ef85c76a36bdcef4811d411ac0f09219803377f5b7b9bd0cf2a5258cc826cea':
        raise ValueError('bag identity changed')
    files = {str(r4 / n): digest(r4 / n) for n in ['sample_manifest.csv', 'alignments.csv',
              'hessian_statistics.csv', 'PROTOCOL.md', 'statistics_core.py', 'archive_hashes.json']}
    result = dict(bag_path=str(bag), bag_sha256=bag_sha, r4_archived_files_verified=len(archived),
                  frozen_pairs=len(samples), raw_pcd_files_verified=len(paths), files=files,
                  raw_pcd_sha256=paths, gt_read=False, hikvision_access=False)
    with new_output.open('x') as stream:
        json.dump(result, stream, indent=2)
    print('PASS: R4 archive, 200 raw PCD and bag identities; no data re-extraction')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('new_output_json', type=Path)
    verify(parser.parse_args().new_output_json)
