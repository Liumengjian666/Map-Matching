#!/usr/bin/env python3
"""Display-only GT math/time tests; no estimator or ROS master is started."""
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET

import numpy as np
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('gt_view', PACKAGE / 'scripts/dataset_gt_path_publisher.py')
GT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(GT)
CONFIG = yaml.safe_load((PACKAGE / 'config/corridor01_gt_visualization.yaml').read_text())


class GroundTruthVisualizationTest(unittest.TestCase):
    def test_frozen_alignment_and_real_extrinsic_not_refitted(self):
        source = PACKAGE.parents[1] / CONFIG['alignment_source']
        self.assertEqual(hashlib.sha256(source.read_bytes()).hexdigest(), CONFIG['alignment_source_sha256'])
        frozen = json.loads(source.read_text())
        alignment = GT.transform(CONFIG['T_gt_map'])
        np.testing.assert_array_equal(alignment[:3, :3], frozen['fit_transform_rotation'])
        np.testing.assert_array_equal(alignment[:3, 3], frozen['fit_transform_translation_m'])
        p18 = yaml.safe_load((PACKAGE / 'config/p18_corridor01_ssd_map.yaml').read_text())
        extrinsic = GT.transform(CONFIG['T_imu_lidar'])
        np.testing.assert_array_equal(extrinsic[:3, :3].reshape(-1),
                                      p18['deskew']['T_imu_lidar_rotation_row_major'])
        np.testing.assert_array_equal(extrinsic[:3, 3], p18['deskew']['T_imu_lidar_translation_m'])

    def test_transform_direction_and_lever_arm(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'gt.txt'
            payload = b'1517157219.794140100 2 3 4 0 0 0 1\n'
            path.write_bytes(payload)
            alignment = GT.transform(CONFIG['T_gt_map'])
            extrinsic = GT.transform(CONFIG['T_imu_lidar'])
            sample = GT.load_samples(path, hashlib.sha256(payload).hexdigest(), alignment, extrinsic)[0]
            self.assertEqual(sample[0], 1517157219794140100)
            matrix = GT.quaternion_matrix(sample[2]); matrix[:3, 3] = sample[1]
            recovered = alignment @ matrix @ np.linalg.inv(extrinsic)
            expected = np.eye(4); expected[:3, 3] = [2, 3, 4]
            np.testing.assert_allclose(recovered, expected, atol=1e-12, rtol=0)

    def test_cursor_never_reveals_future_samples(self):
        cursor = GT.ClockCursor([100, 200, 300])
        for stamp, expected in ((99, -1), (100, 0), (150, 0), (200, 1), (299, 1), (300, 2)):
            index, _, _ = cursor.advance(stamp)
            self.assertEqual(index, expected)
            self.assertTrue(all(value <= stamp for value in cursor.stamps[:index + 1]))

    def test_pause_end_and_rewind(self):
        cursor = GT.ClockCursor([100, 200, 300])
        self.assertEqual(cursor.advance(250), (1, False, True))
        self.assertEqual(cursor.advance(250), (1, False, False))
        self.assertEqual(cursor.advance(240), (1, True, True))
        self.assertEqual(cursor.advance(50), (-1, True, True))
        self.assertEqual(cursor.advance(1000), (2, False, True))
        self.assertEqual(cursor.advance(2000), (2, False, False))

    def test_real_gt_support_and_finite_output(self):
        samples = GT.load_samples(CONFIG['gt_file'], CONFIG['gt_sha256'],
                                  GT.transform(CONFIG['T_gt_map']), GT.transform(CONFIG['T_imu_lidar']))
        self.assertEqual(len(samples), 1385)
        self.assertEqual(samples[0][0], 1517157219794140100)
        self.assertEqual(samples[-1][0], 1517157498957649946)
        for _, position, quaternion in samples:
            self.assertTrue(np.isfinite(position).all())
            self.assertAlmostEqual(np.linalg.norm(quaternion), 1, places=12)

    def test_corridor02_inverse_active_yaml_not_identity_or_transpose(self):
        config = yaml.safe_load((PACKAGE / 'config/corridor02_gt_visualization.yaml').read_text())
        self.assertTrue(config['enabled'])
        self.assertEqual(config['alignment_mode'], 'YAML_GT_FROM_MAP')
        self.assertTrue(config['publish_full_path'])
        self.assertEqual(config['frame_id'], 'map')
        source = Path(config['alignment_source'])
        self.assertEqual(hashlib.sha256(source.read_bytes()).hexdigest(),
                         config['alignment_source_sha256'])
        # SafeLoader handles OpenCV matrices after removing its nonstandard directive.
        class OpenCvLoader(yaml.SafeLoader):
            pass
        OpenCvLoader.add_constructor('tag:yaml.org,2002:opencv-matrix',
                                     lambda loader, node: loader.construct_mapping(node, deep=True))
        active = yaml.load('\n'.join(source.read_text().splitlines()[1:]), Loader=OpenCvLoader)
        matrix = GT.transform(config['T_gt_map'], rotation_tolerance=1e-5)
        np.testing.assert_array_equal(matrix[:3, :3].ravel(),
                                      active['extrinsicRotation_world_darpa']['data'])
        np.testing.assert_array_equal(matrix[:3, 3], active['extrinsicTranslation_world_darpa']['data'])
        samples = GT.load_samples(config['gt_file'], config['gt_sha256'], matrix, np.eye(4))
        self.assertEqual(len(samples), 5522)
        self.assertEqual(samples[0][0], 1645999726984117000)
        self.assertEqual(samples[-1][0], 1646000619653331000)
        raw = np.loadtxt(config['gt_file'])
        expected = np.linalg.solve(matrix[:3, :3], (raw[:, 1:4] - matrix[:3, 3]).T).T
        np.testing.assert_allclose([sample[1] for sample in samples], expected, atol=1e-12, rtol=0)
        np.testing.assert_allclose([np.linalg.norm(sample[2]) for sample in samples], 1, atol=1e-12)

    def test_full_reference_is_complete_timeless_and_independent_of_rewind_path(self):
        samples = [(1000000000, [1, 2, 3], [0, 0, 0, 1]),
                   (2000000000, [4, 5, 6], [0, 0, 0, 1])]
        poses = [GT.make_pose(sample, 'map') for sample in samples]
        full = GT.full_reference_path(poses, 'map')
        poses.clear()
        self.assertEqual(full.header.frame_id, 'map')
        self.assertEqual(full.header.stamp.to_nsec(), 0)
        self.assertEqual([pose.header.stamp.to_nsec() for pose in full.poses],
                         [1000000000, 2000000000])

    def test_rounded_rotation_validation_does_not_accept_bad_transforms(self):
        config = dict(rotation_row_major=[1, 0, 0, 0, 1, 0, 0, 0, -1], translation_m=[0, 0, 0])
        with self.assertRaises(ValueError):
            GT.transform(config, rotation_tolerance=1e-5)
        config['rotation_row_major'][-1] = float('nan')
        with self.assertRaises(ValueError):
            GT.transform(config, rotation_tolerance=1e-5)

    def test_yaml_switch_defaults_and_native_launch(self):
        for dataset in ('corridor01', 'corridor02'):
            params = yaml.safe_load(subprocess.check_output(
                ['roslaunch', '--dump-params', 'dog_prior_map_localization',
                 dataset + '_ground_truth.launch'], text=True, timeout=30))
            config = yaml.safe_load((PACKAGE / 'config' / (dataset + '_gt_visualization.yaml')).read_text())
            self.assertEqual(params['/' + dataset + '_gt_visualizer/enabled'], config['enabled'])

    def test_corridor02_visible_defaults_without_modal_dialog_or_fake_tf(self):
        launch = ET.parse(PACKAGE / 'launch/corridor02_ground_truth.launch')
        rviz = launch.find("node[@name='corridor02_gt_rviz']")
        environment = {entry.get('name'): entry.get('value') for entry in rviz.findall('env')}
        self.assertEqual(environment['DISABLE_ROS1_EOL_WARNINGS'], '1')
        self.assertEqual(environment['LD_PRELOAD'], '/lib/x86_64-linux-gnu/libusb-1.0.so.0')
        self.assertEqual({node.get('type') for node in launch.iter('node')},
                         {'dataset_gt_path_publisher.py', 'rviz', 'pcd_to_pointcloud'})
        map_node = launch.find("node[@name='corridor02_prior_map']")
        self.assertEqual(map_node.find("param[@name='use_sim_time']").get('value'), 'false')
        self.assertEqual(map_node.find("remap[@from='/use_sim_time']").get('to'),
                         '/corridor02_prior_map/use_sim_time')
        manager = yaml.safe_load((PACKAGE / 'rviz/corridor02_ground_truth.rviz').read_text())[
            'Visualization Manager']
        self.assertEqual(manager['Global Options']['Fixed Frame'], 'map')
        grid = next(display for display in manager['Displays'] if display['Class'] == 'rviz/Grid')
        self.assertTrue(grid['Enabled'])
        self.assertEqual(grid['Reference Frame'], '<Fixed Frame>')
        self.assertEqual(manager['Views']['Current']['Distance'], 850)
        self.assertTrue(any(display.get('Topic') == '/dog_livo/gt_path_full'
                            for display in manager['Displays']))
        self.assertTrue(any(display.get('Topic') == '/dog_livo/prior_map'
                            for display in manager['Displays']))

    def test_rejects_nonfinite_nonmonotonic_and_wrong_input(self):
        for payload in (b'1.0 nan 0 0 0 0 0 1\n', b'1.0 0 0 0 0 0 0 0\n',
                        b'1.0 0 0 0 0 0 0 1\n1.0 0 0 0 0 0 0 1\n',
                        b'1.0000000001 0 0 0 0 0 0 1\n'):
            with self.subTest(payload=payload), tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / 'gt.txt'; path.write_bytes(payload)
                with self.assertRaises(ValueError):
                    GT.load_samples(path, hashlib.sha256(payload).hexdigest(), np.eye(4), np.eye(4))
        with self.assertRaises(ValueError):
            GT.load_samples(CONFIG['gt_file'], 'wrong_hash', np.eye(4), np.eye(4))

if __name__ == '__main__':
    unittest.main(verbosity=2)
