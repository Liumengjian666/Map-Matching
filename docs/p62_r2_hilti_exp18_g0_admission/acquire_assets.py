#!/usr/bin/python3
"""Five fixed assets, one GET per missing file, internal disk only. No replay."""
import hashlib
import json
import os
import signal
import shutil
import time
import urllib.request
from pathlib import Path
from urllib.parse import urlsplit

REVISION = 'e62017f907007fdc5ab8c721842e4ae7359d7f49'
ROOT = Path('/home/jian/livox_ws/p62_r2_hilti_exp18_g0_data')
CACHE = Path('/home/jian/livox_ws/p62_r1_source_cache')
ASSETS = [
    ('rosbags/exp18_corridor_lower_gallery_2.bag', 8657654860,
     '98373b52f207b9ace3914792b5604a6642f89441190cabca6acbaf1d17249280', b'#ROSBAG V2.0\n', None),
    ('laser_scans/sheldonian_full.e57', 5447128064,
     '46575d3eaf3169a24e24ba0013cc48610125ac842d6745e5e8156c6aecc59756', b'ASTM-E57', None),
    ('ground_truth/exp18_corridor_lower_gallery_2_imu.txt', 88743,
     '138def6a40884a87372a4f18938cce5180d6aa7ad4e705349abf47ac4ec9fa32', b'', 'hilti_exp18_6dof_schema.txt'),
    ('ground_truth/exp18_corridor_lower_gallery_2_imu_3dof.txt', 185,
     '13f963c4a2e9d583532051b581a19cf73ade0aadc83f530ea3e66c6d52036dec', b'', 'hilti_exp18_3dof_schema.txt'),
    ('calibration/calibration_files/lidar_calibration.yaml', 747,
     'ea5127989317270d83d9a701318d7e284031a21bd9cbf466b8f9f169547db43c', b'', 'hilti_lidar_calibration.yaml'),
]


def identity(path):
    digest = hashlib.sha256()
    with path.open('rb') as handle:
        for chunk in iter(lambda: handle.read(4 * 1024 * 1024), b''):
            digest.update(chunk)
    return {'bytes': path.stat().st_size, 'sha256': digest.hexdigest()}


def verify(path, size, digest):
    actual = identity(path)
    if actual != {'bytes': size, 'sha256': digest}:
        raise RuntimeError('ASSET_IDENTITY_MISMATCH:' + str(path))
    return actual


def available(path='/home/jian/livox_ws'):
    return shutil.disk_usage(str(path)).free


def internal_directory(path):
    base = Path('/home/jian/livox_ws').resolve()
    if ROOT.is_symlink() or ROOT.parent.resolve() != base:
        raise RuntimeError('NON_INTERNAL_ROOT')
    nodes = (path,) + tuple(path.parents)
    if base not in nodes:
        raise RuntimeError('NON_INTERNAL_DIRECTORY')
    for create in (False, True):
        if create:
            path.mkdir(parents=True, exist_ok=True)
        for node in nodes:
            if node == base:
                break
            if node.is_symlink() or (node.exists() and node.stat().st_dev != base.stat().st_dev):
                raise RuntimeError('NON_INTERNAL_DIRECTORY:' + str(node))


def deadline(signum, frame):
    raise TimeoutError('BOUNDED_FILE_WALL_TIME')


def main():
    start_free = available()
    if start_free < 48000000000:
        raise RuntimeError('G0_BLOCKED_STORAGE:' + str(start_free))
    internal_directory(ROOT)
    receipt_path = ROOT / 'acquisition.json'
    if receipt_path.exists():
        raise RuntimeError('RECEIPT_EXISTS_NO_AUTOMATIC_RERUN')
    record = {'revision': REVISION, 'initial_available_bytes': start_free,
              'minimum_start_bytes': 48000000000, 'safety_reserve_bytes': 20 * 1024**3,
              'attempt_limit_per_file': 1, 'assets': []}
    with receipt_path.open('x') as receipt:
        json.dump(record, receipt, indent=2)
    try:
        for relative, size, digest, magic, cache_name in ASSETS:
            url = 'https://huggingface.co/datasets/Hilti-Research/hilti-slam-challenge-2022/resolve/' + REVISION + '/' + relative
            destination = ROOT / 'assets' / relative
            row = {'relative': relative, 'url': url, 'expected_bytes': size,
                   'expected_sha256': digest, 'path': str(destination), 'network_attempts': 0}
            record['assets'].append(row)
            internal_directory(destination.parent)
            if destination.is_symlink():
                raise RuntimeError('SYMLINK_ASSET')
            if destination.exists():
                row.update(verify(destination, size, digest)); row['status'] = 'VERIFIED_EXISTING'
                continue
            temporary = ROOT / 'staging' / (Path(relative).name + '.part')
            internal_directory(temporary.parent)
            if temporary.exists() or temporary.is_symlink():
                raise RuntimeError('PARTIAL_EXISTS_NO_OVERWRITE:' + str(temporary))
            begin = time.monotonic()
            signal.signal(signal.SIGALRM, deadline)
            signal.setitimer(signal.ITIMER_REAL, 1800)
            cache_path = CACHE / cache_name if cache_name else None
            if cache_path is not None and cache_path.exists():
                verify(cache_path, size, digest)
                source = cache_path.open('rb')
                row['source'] = str(cache_path)
            else:
                row['network_attempts'] = 1
                source = urllib.request.urlopen(urllib.request.Request(url, headers={'User-Agent': 'P62-G0/1.0'}), timeout=30)
                row.update(http_status=source.status, final_host=urlsplit(source.url).hostname,
                           content_type=source.headers.get('Content-Type'),
                           content_length=source.headers.get('Content-Length'))
                if source.status != 200:
                    source.close()
                    raise RuntimeError('NON_200_RESPONSE')
            count = 0
            try:
                with source, temporary.open('xb') as target:
                    while True:
                        if time.monotonic() - begin > 1800:
                            raise TimeoutError('BOUNDED_FILE_WALL_TIME')
                        if available() < 20 * 1024**3:
                            raise RuntimeError('SAFETY_RESERVE_REACHED')
                        chunk = source.read(1024 * 1024)
                        if not chunk:
                            break
                        if count == 0 and not chunk.startswith(magic):
                            raise RuntimeError('INVALID_FILE_MAGIC')
                        count += len(chunk)
                        if count > size:
                            raise RuntimeError('RESPONSE_TOO_LARGE')
                        if available(temporary.parent) - len(chunk) < 20 * 1024**3:
                            raise RuntimeError('SAFETY_RESERVE_REACHED')
                        target.write(chunk)
                        if count // (256 * 1024**2) != (count - len(chunk)) // (256 * 1024**2):
                            print(json.dumps({'file': relative, 'bytes': count, 'wall_s': time.monotonic()-begin}), flush=True)
                    target.flush(); os.fsync(target.fileno())
            finally:
                signal.setitimer(signal.ITIMER_REAL, 0)
                row.update(transferred_bytes=count, wall_s=time.monotonic()-begin)
            row.update(verify(temporary, size, digest))
            temporary.chmod(0o444)
            os.link(str(temporary), str(destination))  # exclusive publish; cannot overwrite
            row.update(status='VERIFIED_PUBLISHED', temporary_preserved=str(temporary),
                       mean_bytes_per_second=count/max(row['wall_s'], 1e-9))
            print(json.dumps(row), flush=True)
    except Exception as error:
        record['status'] = 'STOPPED'
        record['error'] = type(error).__name__ + ':' + str(error)
        raise
    else:
        record['status'] = 'ALL_FIVE_LOCAL_IDENTITIES_VERIFIED'
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0)
        record['final_available_bytes'] = available()
        with receipt_path.open('w') as receipt:
            json.dump(record, receipt, indent=2); receipt.write('\n')


if __name__ == '__main__':
    main()
