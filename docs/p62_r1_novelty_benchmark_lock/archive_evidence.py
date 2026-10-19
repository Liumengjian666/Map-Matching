#!/usr/bin/python3
"""Archive receipts and hashes only. Never reads sensor bags or production data."""
import csv
import hashlib
import json
import subprocess
from pathlib import Path

HERE = Path(__file__).resolve().parent
CACHE = Path('/home/jian/livox_ws/p62_r1_source_cache')
RECEIPTS = ('capture_initial.json', 'capture_additional.json')


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    records = []
    for name in RECEIPTS:
        for row in json.loads((CACHE / name).read_text()):
            if row['status'] == 200:
                path = Path(row['path'])
                if path.stat().st_size != row['bytes'] or sha(path) != row['sha256']:
                    raise RuntimeError('CACHE_IDENTITY_CHANGED:' + str(path))
            records.append(row)
    source = {
        'scope': 'Finite primary-source receipts; papers and reference schema stay outside Git',
        'date': '2026-10-19',
        'sources': records,
        'additional_web_reading': [
            {'url': 'https://onlinelibrary.wiley.com/doi/full/10.1002/rob.22392',
             'status': 'METHODS_READ_VIA_WEB_TOOL; local capture HTTP403'},
            {'url': 'https://arxiv.org/html/2302.06843',
             'status': 'METHODS_4.4_4.5_READ_VIA_WEB_TOOL'},
            {'url': 'https://openaccess.thecvf.com/content/CVPR2023/html/Zhang_3D_Registration_With_Maximal_Cliques_CVPR_2023_paper.html',
             'status': 'FETCH_FAILED; indexed abstract only'},
        ],
        'observed_official_code_heads': {
            'Hilti-Research/hilti-slam-challenge-2022': 'e4aacf7c111ab832a0bdffe3f069255454068540',
            'DCSI2022/DALI_SLAM': '4d9b74655ec5896255d5f9ba75b61a52817a4305',
            'WHU-USI3DV/RegScorer': 'c907b16d75eb89374c4a837c1dbc0c535ac42329',
        },
        'mutable_source_warning': 'Some fetched URLs used main; captured bytes are frozen by SHA, not claimed to be immutable URLs.',
    }
    (HERE / 'source_manifest.json').write_text(json.dumps(source, indent=2) + '\n')
    with (HERE / 'LITERATURE_COMPARISON.csv').open(newline='') as handle:
        rows = list(csv.DictReader(handle))
    if len(rows) != 14 or any(None in row or any(v is None for v in row.values()) for row in rows):
        raise RuntimeError('LITERATURE_TABLE_SCHEMA')
    if [row['id'] for row in rows] != list('ABCDEFGHIJKLMN'):
        raise RuntimeError('LITERATURE_TABLE_IDENTITIES')
    root = HERE.parents[1]
    head = subprocess.check_output(['git', '-C', str(root), 'rev-parse', 'HEAD'], text=True).strip()
    outputs = [p for p in sorted(HERE.rglob('*')) if p.is_file()
               and '__pycache__' not in p.parts
               and p.name not in ('analysis_manifest.json', 'checksums.sha256')]
    manifest = {
        'analysis_tree_head_at_archive_generation': head,
        'self_reference_policy': 'Commit identity recorded in final handoff; this manifest excludes itself and checksums.',
        'artifacts': [{'path': str(p.relative_to(root)), 'bytes': p.stat().st_size,
                       'sha256': sha(p)} for p in outputs],
        'raw_bag_downloads': 0, 'sensor_file_reads': 0, 'hdd_reads': 0,
        'reference_schema_files_read': 2, 'reference_error_evaluations': 0,
        'new_ndt_align': 0, 'ros_localization_nodes': 0, 'ekf_runs': 0,
        'production_code_changes': 0,
    }
    (HERE / 'analysis_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    checksum_files = outputs + [HERE / 'analysis_manifest.json']
    (HERE / 'checksums.sha256').write_text(''.join(
        '{}  {}\n'.format(sha(p), p.relative_to(HERE)) for p in checksum_files))
    print(json.dumps({'literature_rows': len(rows), 'source_receipts': len(records),
                      'artifacts': len(checksum_files), 'status': 'VERIFIED'}))


if __name__ == '__main__':
    main()
