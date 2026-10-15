#!/usr/bin/env python3
"""Offline GT-assisted DISPLAY calibration; never an estimator initializer."""
import argparse
import hashlib
import json
from pathlib import Path
import resource
import sys
import time

import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree
import yaml

WORKSPACE = Path(__file__).resolve().parents[2]
PACKAGE = WORKSPACE / 'src/dog_prior_map_localization'
ASSET = Path('/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01')
MAP = WORKSPACE / '.p18_map_recovery/assets/recovery.oEmedI/corridor01_map_normalized.pcd'
TRAIN_SECONDS = (20, 80, 150, 220)
CHECK_SECONDS = (10, 60, 120, 180, 240)


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def matrix(values):
    result = np.eye(4)
    result[:3, :3] = np.asarray(values['rotation_row_major']).reshape(3, 3)
    result[:3, 3] = values['translation_m']
    return result


def cloud(points):
    result = o3d.geometry.PointCloud()
    result.points = o3d.utility.Vector3dVector(points)
    return result


def moved(points, transform):
    return points @ transform[:3, :3].T + transform[:3, 3]


def rigid(transform):
    rotation = transform[:3, :3]
    return bool(np.isfinite(transform).all()
                and np.allclose(transform[3], [0, 0, 0, 1], atol=1e-10, rtol=0)
                and np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-8, rtol=0)
                and abs(np.linalg.det(rotation) - 1) < 1e-8)


def distances(points, transform, tree):
    values = tree.query(moved(points, transform))[0]
    return dict(median_m=float(np.median(values)), p90_m=float(np.percentile(values, 90)),
                coverage_0_5m=float(np.mean(values <= .5)),
                coverage_2m=float(np.mean(values <= 2)))


def acceptable(stats):
    return (stats['median_m'] <= 1 and stats['p90_m'] <= 3
            and stats['coverage_0_5m'] >= .5 and stats['coverage_2m'] >= .8)


def nearest_headers(messages, targets):
    """Global header-time nearest scans; bag record time is not a selection key."""
    selected = {}
    previous = -1
    for _, message, record in messages:
        ns = message.header.stamp.to_nsec()
        if ns <= previous:
            raise ValueError('Unordered sensor headers')
        previous = ns
        for seconds, target_ns in targets.items():
            distance = abs(ns - target_ns)
            if seconds not in selected or distance < selected[seconds][0]:
                selected[seconds] = (distance, message, record.to_nsec())
    if set(selected) != set(targets):
        raise ValueError('Missing fixed scan targets')
    return selected


def main(output, single_turn_anchor=False):
    import rosbag
    sys.path.insert(0, str(PACKAGE / 'scripts/p32_gt_frame'))
    from prepare import load_gt, at
    from prepare_probe import cloud_array

    output.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    config_path = PACKAGE / 'config/corridor01_gt_visualization.yaml'
    config = yaml.safe_load(config_path.read_text())
    gt_path = ASSET / 'gt/corridor01_gt.txt'
    if sha(MAP) != '103a01b2c89ca2adbd2b2e22256acba6e91f40b1028857c2103f08c6c3eb8f8f':
        raise ValueError('Frozen map SHA256 mismatch')
    if sha(gt_path) != config['gt_sha256']:
        raise ValueError('GT SHA256 mismatch')
    extrinsic = matrix(config['T_imu_lidar'])
    if not rigid(extrinsic):
        raise ValueError('Invalid RC2 extrinsic')
    train_seconds = (150,) if single_turn_anchor else TRAIN_SECONDS
    check_seconds = (45, 100, 160, 200, 260) if single_turn_anchor else CHECK_SECONDS
    old_alignment = np.linalg.inv(matrix(config['T_gt_map']))
    gt = load_gt(gt_path)
    bag_path = ASSET / 'raw/Long_Corridor_Rosbag/corridor01_decoded_raw_timed_full.bag'
    scans, selection = {}, []
    with rosbag.Bag(str(bag_path)) as bag:
        targets = {seconds: gt[0][0] + seconds * 10**9
                   for seconds in sorted(train_seconds + check_seconds)}
        selected = nearest_headers(bag.read_messages(
            topics=['/p10/corridor01/raw_timed_points']), targets)
        for seconds, target in targets.items():
            _, msg, record_ns = selected[seconds]
            ns = msg.header.stamp.to_nsec()
            if abs(ns - target) > 60_000_000 or msg.header.frame_id != 'cmu_rc2_velodyne':
                raise ValueError('Scan identity mismatch')
            raw = cloud_array(msg)
            if not np.isfinite(raw[:, 3]).all() or raw[:, 3].min() < 0 or raw[:, 3].max() > .15:
                raise ValueError('Point time contract')
            xyz = raw[:, :3]
            ranges = np.linalg.norm(xyz, axis=1)
            xyz = xyz[np.isfinite(xyz).all(axis=1) & (ranges >= .5) & (ranges <= 80)]
            if len(xyz) < 1000:
                raise ValueError('Insufficient real points')
            pose, bracket = at(gt, ns)
            scans[seconds] = moved(xyz, pose @ extrinsic)
            selection.append(dict(target_GT_seconds=seconds, scan_ns=str(ns), record_ns=str(record_ns),
                role='TRAIN' if seconds in train_seconds else 'HOLDOUT',
                points=len(xyz), payload_SHA256=hashlib.sha256(msg.data).hexdigest(),
                GT_bracket=bracket))
    receipt = dict(display_only=True, GT_assisted=True, GT_used_for_localization=False,
        train_seconds=train_seconds, holdout_seconds=check_seconds, scans=selection,
        map_SHA256=sha(MAP), GT_SHA256=sha(gt_path), script_SHA256=sha(__file__),
        extrinsic=extrinsic.tolist(), T_old_map_GT=old_alignment.tolist(),
        method='Open3D 0.19 FPFH + one RANSAC + two point-to-plane ICP scales',
        voxel_coarse_m=1., RANSAC_distance_m=1.5, RANSAC_max_iterations=100000,
        RANSAC_seed=7, ICP_voxels_m=[.5, .2], ICP_distances_m=[1.5, .6],
        ICP_max_iterations_per_scale=100, no_scaling=True, no_reflection=True,
        validation='all train and heldout scans: median<=1m, P90<=3m, coverage0.5m>=50%, coverage2m>=80%',
        source_mode='raw finite XYZ, range0.5-80m, rigid header-time GT; NO deskew',
        bag_full_SHA256_recomputed=False)
    (output / 'selection.json').write_text(json.dumps(receipt, indent=2) + '\n')
    target = o3d.io.read_point_cloud(str(MAP))
    if len(target.points) != 338210 or not np.isfinite(np.asarray(target.points)).all():
        raise ValueError('Invalid frozen target')
    source = cloud(np.concatenate([scans[t] for t in train_seconds]))
    o3d.io.write_point_cloud(str(output / 'training_GT_world.pcd'), source)
    coarse = [item.voxel_down_sample(1.) for item in (source, target)]
    features = []
    for item in coarse:
        item.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=2., max_nn=50))
        features.append(o3d.pipelines.registration.compute_fpfh_feature(
            item, o3d.geometry.KDTreeSearchParamHybrid(radius=5., max_nn=100)))
    reg = o3d.pipelines.registration
    o3d.utility.random.seed(7)
    print('FPFH ready: ' + str([len(item.points) for item in coarse]), flush=True)
    initial = reg.registration_ransac_based_on_feature_matching(
        coarse[0], coarse[1], features[0], features[1], True, 1.5,
        reg.TransformationEstimationPointToPoint(False), 3,
        [reg.CorrespondenceCheckerBasedOnEdgeLength(.9),
         reg.CorrespondenceCheckerBasedOnDistance(1.5)],
        reg.RANSACConvergenceCriteria(100000, .999))
    transform = initial.transformation.copy()
    if not rigid(transform):
        raise ValueError('Non-rigid RANSAC transform')
    stages = [dict(stage='RANSAC', fitness=initial.fitness, rmse=initial.inlier_rmse,
                   transform=transform.tolist())]
    print('RANSAC: ' + str(stages[-1]), flush=True)
    for voxel, maximum in [(.5, 1.5), (.2, .6)]:
        fine_source, fine_target = [item.voxel_down_sample(voxel) for item in (source, target)]
        fine_target.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel*2, max_nn=50))
        result = reg.registration_icp(fine_source, fine_target, maximum, transform,
            reg.TransformationEstimationPointToPlane(reg.TukeyLoss(k=maximum)),
            reg.ICPConvergenceCriteria(max_iteration=100))
        transform = result.transformation.copy()
        stages.append(dict(stage='ICP', voxel_m=voxel, fitness=result.fitness,
                           rmse=result.inlier_rmse, transform=transform.tolist()))
        print('ICP: ' + str(stages[-1]), flush=True)
    if not rigid(transform):
        raise ValueError('Non-rigid estimated transform')
    # Holdouts are accessed for evaluation only, never added to source or optimized.
    tree = cKDTree(np.asarray(target.points))
    evaluation = []
    for seconds in sorted(scans):
        old, new = [distances(scans[seconds], T, tree) for T in (old_alignment, transform)]
        evaluation.append(dict(target_GT_seconds=seconds,
            role='TRAIN' if seconds in train_seconds else 'HOLDOUT',
            before=old, after=new, geometry_pass=acceptable(new)))
    passed = all(row['geometry_pass'] for row in evaluation)
    result = dict(receipt, status='DISPLAY_ALIGNMENT_GEOMETRY_PASS' if passed else 'DISPLAY_ALIGNMENT_REJECTED',
        T_map_GT=transform.tolist(), stages=stages, evaluation=evaluation,
        elapsed_s=time.monotonic()-started, peak_RSS_KiB=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss)
    (output / 'alignment.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--single-turn-anchor', action='store_true',
                        help='One bounded diagnostic fallback after the multi-scan attempt fails')
    main(**vars(parser.parse_args()))
