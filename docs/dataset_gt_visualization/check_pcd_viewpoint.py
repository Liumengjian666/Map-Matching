#!/usr/bin/env python3
"""Read-only NN diagnosis of one metadata-derived composition; no registration.

Requires the unchanged local P32-R5 cache and existing Open3D/SciPy. Emits JSON
on stdout. GT is used only for offline coordinate diagnosis, never initialization.
"""
import hashlib
import importlib.util
import json
from pathlib import Path
import time

import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree
import yaml

workspace = Path(__file__).resolve().parents[2]
package = workspace / 'src/dog_prior_map_localization'
spec = importlib.util.spec_from_file_location('gt_display', package / 'scripts/dataset_gt_path_publisher.py')
display = importlib.util.module_from_spec(spec)
spec.loader.exec_module(display)
config = yaml.safe_load((package / 'config/corridor02_gt_visualization.yaml').read_text())
frozen = json.loads((workspace / 'docs/p32_r5_corridor02_gt_frame_test/input_freeze.json').read_text())['selection']
root = workspace / '.p32_r5_corridor02_gt_frame_test/input'
for filename, expected in frozen['files_SHA256'].items():
    if hashlib.sha256(Path(filename).read_bytes()).hexdigest() != expected:
        raise ValueError('Frozen diagnostic cache mismatch: ' + filename)
tick = time.monotonic()
alignment = display.pcd_yaml_alignment(config['T_world_gt'], config['reference_map_file'],
                                      config['reference_map_sha256'])
samples = display.load_samples(config['gt_file'], config['gt_sha256'], alignment, np.eye(4))
cloud = o3d.io.read_point_cloud(config['reference_map_file'])
points = np.asarray(cloud.points)
if len(points) != 5223422 or not np.isfinite(points).all():
    raise ValueError('Invalid map')
# Full-resolution target, not the old PCL 0.2m voxel target; reported separately.
tree = cKDTree(points)
poses = np.fromfile(root / 'poses.f64', dtype='<f8').reshape(5, 4, 4)
extrinsic = np.fromfile(root / 'T_I_L.f64', dtype='<f8').reshape(4, 4)
rows = []
for frame, pose in zip(frozen['selected'], poses):
    source = np.fromfile(root / frame['file'], dtype='<f4').reshape(-1, 3)
    if len(source) != frame['points'] or not np.isfinite(source).all():
        raise ValueError('Invalid real scan')
    transform = np.linalg.inv(alignment) @ pose @ extrinsic
    moved = source @ transform[:3, :3].T + transform[:3, 3]
    distances = tree.query(moved)[0]
    rows.append(dict(frame_index=frame['index'], scan_ns=frame['scan_ns'], points=len(source),
                     nn_median_m=float(np.median(distances)), nn_p90_m=float(np.percentile(distances, 90)),
                     coverage_0_5m=float(np.mean(distances <= .5)),
                     coverage_2m=float(np.mean(distances <= 2)),
                     coverage_8m=float(np.mean(distances <= 8)), T_rawXYZ_lidar=transform.tolist()))
positions = np.asarray([sample[1] for sample in samples])
print(json.dumps(dict(status='PCD_VIEWPOINT_FORWARD_YAML_GEOMETRICALLY_SUPPORTED',
    source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    map_sha256=config['reference_map_sha256'], gt_sha256=config['gt_sha256'],
    yaml_sha256=config['alignment_source_sha256'], frozen_cache_hashes_verified=True,
    target='full-resolution raw PCD XYZ; Open3D read + SciPy cKDTree; no voxel',
    T_rawXYZ_GT=np.linalg.inv(alignment).tolist(), map_points=len(points),
    gt_points=len(samples), gt_first=positions[0].tolist(),
    gt_bbox=[positions.min(0).tolist(), positions.max(0).tolist()], scans=rows,
    gt_assisted_diagnostic=True, gt_used_as_runtime_initial_pose=False,
    gt_sensor_origin_certified=False, official_frame_semantics_certified=False,
    ICP_calls=0, NDT_align_calls=0, fitted_transforms=0,
    elapsed_seconds=time.monotonic()-tick), indent=2))
