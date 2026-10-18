#!/usr/bin/env python3
"""Freeze existing internal metadata only. No network or sensor/GT payload reads."""
import hashlib
import json
from pathlib import Path
import re
import shutil
import numpy as np

DOC = Path(__file__).resolve().parent
DATA = Path('/home/jian/geode_p61_data')


def write_new(path, data):
    with path.open('x') as stream:
        json.dump(data, stream, indent=2, allow_nan=False)


def identity(path):
    data = path.read_bytes()
    return {'path': str(path), 'bytes': len(data), 'sha256': hashlib.sha256(data).hexdigest()}


def main():
    receipts = sorted((DATA / 'official_metadata').glob('*.http.json')) + sorted((DOC / 'official').glob('*.http.json'))
    snapshots = [dict(receipt=str(path), **json.loads(path.read_text())) for path in receipts]
    listings = {}
    for path in sorted((DATA / 'official_metadata').glob('*_listing.json')):
        listings[path.stem] = [{'id': r['id'], 'name': r['name'], 'mime': r['mime'],
                               'size_bytes': r['raw_metadata'][13] if len(r['raw_metadata']) > 13
                               and isinstance(r['raw_metadata'][13], int) else None}
                              for r in json.loads(path.read_text())]
    probe = json.loads((DATA / 'bag_access_probe/access_receipt.json').read_text())
    yaml = (DOC / 'official/gamma_config.yaml').read_text()
    match = re.search(r'T_IMU_LiDAR:.*?data: \[(.*?)\]', yaml, re.S)
    matrix = np.array([float(item) for item in match[1].split(',')]).reshape(4, 4)
    calibration = {'field': 'T_IMU_LiDAR', 'raw_matrix': matrix.tolist(),
                   'raw_matrix_inverse': np.linalg.inv(matrix).tolist(),
                   'orthogonality_frobenius': float(np.linalg.norm(matrix[:3, :3].T @ matrix[:3, :3] - np.eye(3))),
                   'determinant': float(np.linalg.det(matrix[:3, :3])),
                   'inverse_residual': float(np.linalg.norm(matrix @ np.linalg.inv(matrix) - np.eye(4))),
                   'runtime_extrinsic_approved': False,
                   'reason': 'Ouster topic, VERT_RES16, IMU topic mismatch, duplicate YAML keys; device/frame applicability unverified'}
    result = {'task': 'P61-GEODE-NATIVE-TUNNEL-ENTRY-R1', 'final_result': 'GEODE_INPUT_CONTRACT_BLOCKED',
              'start_sha': '869f476904130917ea2bc342f19ee0862a135765',
              'metadata_tool_commit': '0ca33208b15d42a879ed0ad31226ba7407b68ea2',
              'official_repository_commit': 'c6e930623d4fed450d7fc50e16e3ffe0288b692b',
              'sequence': 'Shield_tunnel1_gamma', 'fallback_sequence_processed': False,
              'published_bag_size_bytes': 4500527001, 'complete_bag_obtained': False,
              'complete_bag_sha256': None, 'valid_sensor_scans': 0,
              'actual_lidar_message_type': None, 'actual_lidar_frame': None, 'actual_point_time_contract': None,
              'actual_imu_frame': None, 'actual_imu_time_coverage': None,
              'documented_lidar_topic': '/livox/lidar', 'documented_lidar_type': 'livox_ros_driver/CustomMsg',
              'documented_external_imu_topic': '/imu/data', 'documented_external_imu': 'Xsens MTi-30',
              'documented_integrated_imu_topic': '/livox/imu', 'documented_integrated_imu': 'BMI088',
              'documented_not_actual': True, 'extrinsic_approved': False,
              'independent_tunnel_map_found': False, 'map_frame_verified': False,
              'primary_blocker': 'OFFICIAL_BAG_DOWNLOAD_QUOTA_HTML_AFTER_NORMAL_CONFIRMATION',
              'downstream_unclosed_gates': ['GAMMA_CALIBRATION_APPLICABILITY', 'TUNNEL_PRIOR_MAP_AND_FRAME',
                                          'ACTUAL_POINT_TIME_AND_IMU_COVERAGE', 'LEGAL_INITIALIZATION'],
              'bag_initial_requests': 1, 'bag_confirmation_requests': 1, 'automatic_retries': 0,
              'bag_access_probe': probe, 'sensor_or_gt_payload_used': False, 'ndt_align_calls': 0,
              'native_jet_calls': 0, 'ekf_updates': 0, 'oosm_updates': 0, 'deskew_calls': 0,
              'coupled_calls': 0, 'ros_started': False, 'hikvision_access': False,
              'smoke_completed': False, 'localization_accuracy_evaluated': False,
              'gaussian_grid_measured_for_geode': False, 'push_executed': False}
    originals = sorted((DATA / 'official_metadata').glob('*')) + sorted((DATA / 'bag_access_probe').glob('*'))
    manifest = {'official_http_snapshots': snapshots, 'public_folder_listings': listings,
                'internal_evidence_files': [identity(path) for path in originals if path.is_file()],
                'official_small_assets': [identity(path) for path in (DOC / 'official').glob('*') if path.is_file()],
                'official_dataset_checksum_available': False,
                'hash_scope': 'local evidence bytes only; no complete bag checksum was obtained'}
    write_new(DOC / 'source_manifest.json', manifest)
    write_new(DOC / 'calibration_checks.json', calibration)
    write_new(DOC / 'results.json', result)
    evidence = DOC / 'access_evidence'; evidence.mkdir(exist_ok=False)
    for path in [DATA / 'bag_access_probe/access_receipt.json', DATA / 'bag_access_probe/response_body.bin',
                 DATA / 'official_metadata/bag_download_response.bin', DATA / 'official_metadata/bag_download_response.bin.http.json',
                 DATA / 'official_metadata/gt_download_response.bin', DATA / 'official_metadata/gt_download_response.bin.http.json']:
        shutil.copyfile(path, evidence / path.name)
    print('FROZEN metadata-only entry evidence; no complete sensor data, no smoke')


if __name__ == '__main__':
    main()
