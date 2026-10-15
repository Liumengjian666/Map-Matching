#!/usr/bin/env python3
"""Extract only receipt-verified official camera ZIPs and summarize their data."""
import bisect
import csv
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import statistics
import sys
import time
import zipfile

import cv2

DOC = Path(__file__).resolve().parent
P40 = DOC.parent / 'p40_newer_college_download'
sys.path.insert(0, str(P40))
import download_assets as downloader  # noqa: E402

ROOT = downloader.ROOT
SEQUENCE = ROOT / '01_short_experiment'
PLAN = json.loads((DOC / 'download_plan.json').read_text())
PLAN_BY_PATH = {entry['path']: entry for entry in PLAN['files']}
STATUS_PATH = ROOT / 'downloads/p42_stereo_download_status.json'
OUTPUT = SEQUENCE / 'raw_format/infra1'
PNG_NAME = re.compile(r'infra1_(\d+)_(\d{9})\.png\Z')
RESERVE_BYTES = 30 * 1024 ** 3


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def image_time_ns(name):
    match = PNG_NAME.fullmatch(name)
    if not match:
        raise RuntimeError('Unexpected camera image filename: ' + name)
    nsec = int(match.group(2))
    if nsec >= 1_000_000_000:
        raise RuntimeError('Invalid nanosecond field in: ' + name)
    return int(match.group(1)) * 1_000_000_000 + nsec


def read_offsets(path):
    rows = []
    with path.open(newline='') as stream:
        for row in csv.reader(stream):
            if not row or row[0].strip().startswith('#'):
                continue
            stamp = int(row[0]) * 1_000_000_000 + int(row[1])
            rows.append((stamp, int(row[2])))
    if not rows or any(b[0] <= a[0] for a, b in zip(rows, rows[1:])):
        raise RuntimeError('Invalid official camera time-offset table')
    return rows


def offset_at(stamp, rows):
    times = [row[0] for row in rows]
    index = bisect.bisect_left(times, stamp)
    if index == 0 or index == len(rows):
        return None
    t0, v0 = rows[index - 1]
    t1, v1 = rows[index]
    alpha = (stamp - t0) / (t1 - t0)
    return round(v0 + alpha * (v1 - v0))


def main():
    downloader.check_mount()
    os.chdir(str(ROOT))
    device_id = ROOT.stat().st_dev
    if SEQUENCE.stat().st_dev != device_id or OUTPUT.parent.stat().st_dev != device_id:
        raise RuntimeError('Image extraction destination is not on the external filesystem')
    if OUTPUT.is_symlink() or (OUTPUT.exists() and not OUTPUT.is_dir()):
        raise RuntimeError('Refuse to use unsafe infra1 image output path')
    if STATUS_PATH.is_symlink() or not STATUS_PATH.is_file():
        raise RuntimeError('Missing or unsafe P42 download receipt')

    status = json.loads(STATUS_PATH.read_text())
    if status.get('root') != str(ROOT):
        raise RuntimeError('P42 download receipt belongs to another data root')
    receipts = {}
    for receipt in status.get('files', []):
        path = receipt['path']
        if path in receipts or path not in PLAN_BY_PATH:
            raise RuntimeError('Unexpected or duplicate P42 download receipt')
        entry = PLAN_BY_PATH[path]
        if receipt['drive_id'] != entry['drive_id'] or receipt['bytes'] != entry['bytes']:
            raise RuntimeError('Receipt identity differs from the frozen P42 plan')
        archive = ROOT / path
        if archive.is_symlink() or archive.stat().st_dev != device_id:
            raise RuntimeError('Verified archive left the external filesystem')
        actual_hash = sha256(archive)
        if archive.stat().st_size != entry['bytes'] or actual_hash != receipt['sha256']:
            raise RuntimeError('Archive no longer matches its verified receipt: ' + path)
        receipts[path] = receipt

    available = [entry for entry in PLAN['files']
                 if entry['side'] == 'left/infra1' and entry['path'] in receipts]
    if not available:
        raise RuntimeError('No receipt-verified left-camera archives are available')

    started = time.monotonic()
    archive_summaries = []
    members = []
    names = set()
    timestamps = []
    uncompressed_bytes = 0
    for entry in available:
        archive = ROOT / entry['path']
        with zipfile.ZipFile(archive) as zipped:
            infos = zipped.infolist()
            local = []
            for info in infos:
                name = info.filename
                mode = info.external_attr >> 16
                if (info.is_dir() or Path(name).name != name or '/' in name or '\\' in name or
                        (mode & 0o170000) == 0o120000 or not PNG_NAME.fullmatch(name)):
                    raise RuntimeError('Unsafe or unexpected ZIP member: ' + name)
                if name in names:
                    raise RuntimeError('Duplicate camera image across archives: ' + name)
                names.add(name)
                stamp = image_time_ns(name)
                local.append(info)
                members.append((entry['path'], info, stamp))
                timestamps.append(stamp)
                uncompressed_bytes += info.file_size
            bad_member = zipped.testzip()
            if bad_member is not None:
                raise RuntimeError('ZIP CRC failure in %s: %s' % (entry['path'], bad_member))
            archive_summaries.append({
                'path': entry['path'], 'drive_id': entry['drive_id'],
                'compressed_bytes': archive.stat().st_size,
                'sha256': receipts[entry['path']]['sha256'],
                'zip_crc': 'PASS', 'image_count': len(local),
                'uncompressed_bytes': sum(info.file_size for info in local),
                'first_image_ns': min(image_time_ns(info.filename) for info in local),
                'last_image_ns': max(image_time_ns(info.filename) for info in local),
            })

    free_before = os.statvfs(ROOT).f_bavail * os.statvfs(ROOT).f_frsize
    if free_before < uncompressed_bytes + RESERVE_BYTES:
        raise RuntimeError('Insufficient external space for extraction plus 30 GiB reserve')

    expected_sizes = {info.filename: info.file_size for _, info, _ in members}
    extraction_already_complete = False
    if OUTPUT.exists():
        existing_sizes = {}
        for path in OUTPUT.iterdir():
            if path.is_symlink() or not path.is_file():
                raise RuntimeError('Unexpected item in existing infra1 image directory')
            existing_sizes[path.name] = path.stat().st_size
        if existing_sizes != expected_sizes:
            raise RuntimeError('Existing infra1 directory does not exactly match verified ZIP members')
        extraction_already_complete = True
    else:
        OUTPUT.mkdir(exist_ok=False)
        for entry in available:
            downloader.check_mount(device_id)
            archive = ROOT / entry['path']
            with zipfile.ZipFile(archive) as zipped:
                for info in zipped.infolist():
                    destination = OUTPUT / info.filename
                    with zipped.open(info, 'r') as source, destination.open('xb') as target:
                        shutil.copyfileobj(source, target, 4 * 1024 * 1024)

    files = sorted(OUTPUT.glob('*.png'))
    if len(files) != len(names) or len(expected_sizes) != len(names):
        raise RuntimeError('Extracted file count differs from verified ZIP inventory')
    ordered = sorted(timestamps)
    if len(ordered) != len(set(ordered)):
        raise RuntimeError('Duplicate left-camera image timestamp')
    deltas = [(b - a) / 1e9 for a, b in zip(ordered, ordered[1:])]
    median_period = statistics.median(deltas) if deltas else None
    gaps = sum(delta > median_period * 1.5 for delta in deltas) if median_period else None

    sample_paths = []
    for archive in archive_summaries:
        candidates = [OUTPUT / info.filename for source, info, _ in members
                      if source == archive['path']]
        candidates.sort(key=lambda path: image_time_ns(path.name))
        if not candidates:
            continue
        sample_paths.extend([candidates[0], candidates[len(candidates) // 2], candidates[-1]])
    samples = []
    seen_samples = set()
    for path in sample_paths:
        if path.name in seen_samples:
            continue
        seen_samples.add(path.name)
        image = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
        if image is None or image.size == 0:
            raise RuntimeError('Image decode failed: ' + str(path))
        samples.append({'filename': path.name, 'timestamp_ns': image_time_ns(path.name),
                        'width': int(image.shape[1]), 'height': int(image.shape[0]),
                        'channels': 1 if image.ndim == 2 else int(image.shape[2]),
                        'dtype': str(image.dtype), 'decode': 'PASS'})

    dimensions = sorted({(sample['width'], sample['height'], sample['channels'])
                         for sample in samples})
    if len(dimensions) != 1:
        raise RuntimeError('Decoded sample dimensions or channel counts disagree')

    offset_rows = read_offsets(SEQUENCE / 'time_offsets/time_offsets.csv')
    corrected = []
    for stamp in ordered:
        offset = offset_at(stamp, offset_rows)
        if offset is not None:
            corrected.append(stamp - offset)

    base_checks = json.loads((P40 / 'data_definition_checks.json').read_text())
    lidar = json.loads((P40 / 'timestamp_inventory.json').read_text())
    rs_imu = base_checks['raw_format/realsense_imu/data.csv']
    first_ns, last_ns = ordered[0], ordered[-1]
    lidar_first_ns = lidar['lidar_first_ns']
    lidar_last_ns = lidar['lidar_last_ns']
    lidar_overlap = max(first_ns, lidar_first_ns) < min(last_ns, lidar_last_ns)
    imu_overlap = max(first_ns, rs_imu['first_ns']) < min(last_ns, rs_imu['last_ns'])
    corrected_lidar_overlap = bool(corrected) and max(min(corrected), lidar_first_ns) < min(max(corrected), lidar_last_ns)

    result = {
        'status': 'PARTIAL_LEFT_CAMERA_ARCHIVES_CRC_AND_SAMPLE_DECODE_PASS',
        'dataset': PLAN['dataset'], 'selected_side': 'left/infra1',
        'download_status': status.get('status'), 'download_error': status.get('error'),
        'archive_count': len(archive_summaries), 'archive_summaries': archive_summaries,
        'extraction_already_complete_at_validation': extraction_already_complete,
        'left_image_count': len(files), 'right_image_count': 0,
        'stereo_pair_count': 0, 'right_stream_missing': True,
        'first_image_ns': first_ns, 'last_image_ns': last_ns,
        'duration_s': (last_ns - first_ns) / 1e9,
        'median_frame_period_s': median_period,
        'median_fps': (1.0 / median_period) if median_period else None,
        'mean_fps_over_span': ((len(files) - 1) / ((last_ns - first_ns) / 1e9)) if len(files) > 1 else None,
        'intervals_over_1_5x_median': gaps,
        'duplicate_timestamps': 0, 'image_filename_pattern': 'infra1_<sec>_<nsec>.png',
        'decoded_dimensions_width_height_channels': dimensions[0],
        'sample_image_decodes': samples,
        'all_available_zip_crc_pass': True,
        'uncompressed_bytes': uncompressed_bytes,
        'free_bytes_at_validation_start': free_before,
        'free_bytes_at_validation_end': os.statvfs(ROOT).f_bavail * os.statvfs(ROOT).f_frsize,
        'lidar_time_range_ns': [lidar_first_ns, lidar_last_ns],
        'raw_camera_lidar_time_overlap': lidar_overlap,
        'realsense_imu_time_range_ns': [rs_imu['first_ns'], rs_imu['last_ns']],
        'raw_camera_realsense_imu_time_overlap': imu_overlap,
        'official_camera_offset_table_range_ns': [offset_rows[0][0], offset_rows[-1][0]],
        'camera_frames_with_interpolable_offset': len(corrected),
        'offset_formula': 'camera_corrected = camera_timestamp - time_offset_nsec',
        'offset_corrected_camera_range_ns': [min(corrected), max(corrected)] if corrected else None,
        'offset_corrected_camera_lidar_overlap': corrected_lidar_overlap,
        'offset_not_extrapolated': True,
        'wall_time_s': time.monotonic() - started,
    }
    output_json = DOC / 'image_validation.json'
    output_json.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
