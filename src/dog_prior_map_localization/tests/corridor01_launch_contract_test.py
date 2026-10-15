#!/usr/bin/env python3
"""Offline launch-contract checks; source this workspace's devel/setup.bash first.

Does not start ROS nodes, publish data, or modify the ROS parameter server.
"""

import pathlib
import subprocess
import unittest
import xml.etree.ElementTree as ET

import yaml


PACKAGE = pathlib.Path(__file__).resolve().parents[1]
LAUNCH = PACKAGE / 'launch/corridor01_localization.launch'
MAP = ('/home/jian/livox_ws/p10_corridor01_oosm_r1_ws/'
       '.p18_map_recovery/assets/recovery.oEmedI/corridor01_map_normalized.pcd')


def roslaunch_output(option, *arguments):
    return subprocess.check_output(
        ['roslaunch', option, str(LAUNCH), *arguments],
        text=True, timeout=45)


class Corridor01LaunchContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tree = ET.parse(LAUNCH)
        cls.params = yaml.safe_load(roslaunch_output('--dump-params'))

    def test_control_and_existing_estimator_contract(self):
        expected = {
            '/use_sim_time': True,
            '/coupled_mode': 'CONTROL',
            '/visual_initial_guess_enable': False,
            '/dog_prior_map_ndt/coupled_mode': 'CONTROL',
            '/map/load_in_ekf': False,
            '/map/pcd_fallback_path': MAP,
            '/frames/map_frame': 'camera_init',
            '/topics/imu': '/imu/data',
            '/topics/lidar': '/p10/corridor01/deskewed_points',
            '/deskew/mode': 'ekf_imu_fastlio',
            '/deskew/input_topic': '/p10/corridor01/raw_timed_points',
            '/deskew/output_topic': '/p10/corridor01/deskewed_points',
            '/deskew/reference_time': 'start',
            '/lidar_update/deskew_enable': False,
            '/lidar_update/scan_reference_time': 'start',
            '/lidar_update/ndt_resolution': 0.8,
            '/lidar_update/ndt_max_iterations': 40,
            '/ndt_observation/oosm_enable': True,
            '/ndt_observation/oosm_fixed_lag_enable': True,
            '/ndt_observation/oosm_fixed_lag_sec': 0.15,
            '/ndt_observation/future_deferral_enable': False,
            '/ndt_observation/velocity_blend': 0.6,
            '/dog_prior_map_ndt/visual_initial_guess_enable': False,
            '/output/publish_tf': True,
            '/output/ndt_publish_tf': False,
        }
        for key, value in expected.items():
            with self.subTest(parameter=key):
                self.assertEqual(self.params[key], value)
        for key in (
                '/mature_integration/enable', '/mature_visual/enable',
                '/mature_visual/time_adapter_enable',
                '/filter/experimental_covariance_stability_enable',
                '/filter/experimental_square_root_covariance_enable',
                '/deskew/experimental_scan_local_imu_enable',
                '/deskew/experimental_history_nominal_consistent_enable',
                '/lidar_update/experimental_ekf_translation_fallback_enable',
                '/output/experimental_ndt_source_capture_enable'):
            with self.subTest(disabled_experiment=key):
                self.assertFalse(self.params[key])

    def test_map_publisher_and_no_static_transform(self):
        node = self.tree.find("node[@name='corridor01_prior_map']")
        self.assertEqual(node.get('pkg'), 'pcl_ros')
        self.assertEqual(node.get('type'), 'pcd_to_pointcloud')
        self.assertEqual(node.find('remap').attrib,
                         {'from': 'cloud_pcd', 'to': '/dog_livo/prior_map'})
        self.assertEqual(self.params['/corridor01_prior_map/file_name'], MAP)
        self.assertEqual(self.params['/corridor01_prior_map/frame_id'],
                         self.params['/frames/map_frame'])
        self.assertTrue(self.params['/corridor01_prior_map/latch'])
        self.assertEqual(self.params['/corridor01_prior_map/interval'], 0.0)
        self.assertEqual(node.find('env').attrib,
                         {'name': 'LD_PRELOAD',
                          'value': '/lib/x86_64-linux-gnu/libusb-1.0.so.0'})
        self.assertNotIn('static_transform_publisher', LAUNCH.read_text())

    def test_overrides_and_headless_nodes(self):
        params = yaml.safe_load(roslaunch_output(
            '--dump-params', 'rviz:=false', 'map_path:=/tmp/map with spaces.pcd',
            'raw_points_topic:=/test/timed_points', 'imu_topic:=/test/imu'))
        self.assertEqual(params['/map/pcd_fallback_path'], '/tmp/map with spaces.pcd')
        self.assertEqual(params['/corridor01_prior_map/file_name'],
                         '/tmp/map with spaces.pcd')
        self.assertEqual(params['/deskew/input_topic'], '/test/timed_points')
        self.assertEqual(params['/topics/imu'], '/test/imu')
        nodes = set(roslaunch_output('--nodes', 'rviz:=false').splitlines())
        self.assertEqual(nodes, {'/dog_prior_map_ndt', '/dog_prior_map_ekf',
                                 '/corridor01_prior_map', '/corridor01_gt_visualizer'})

    def test_reused_rviz_configuration(self):
        config = yaml.safe_load((PACKAGE / 'rviz/corridor01_with_gt.rviz').read_text())
        manager = config['Visualization Manager']
        self.assertEqual(manager['Global Options']['Fixed Frame'], 'camera_init')
        prior_map = next(display for display in manager['Displays']
                         if display.get('Name') == 'Prior Map')
        self.assertEqual(prior_map['Topic'], '/dog_livo/prior_map')
        self.assertTrue(prior_map['Enabled'])
        self.assertTrue(any(display.get('Topic') == '/dog_livo/gt_path'
                            for display in manager['Displays']))
        self.assertTrue(any(display.get('Marker Topic') == '/dog_livo/gt_current_pose'
                            for display in manager['Displays']))

    def test_optional_gt_does_not_change_estimator_parameters(self):
        off = yaml.safe_load(roslaunch_output('--dump-params', 'gt_visualization:=false'))
        on = yaml.safe_load(roslaunch_output('--dump-params', 'gt_visualization:=true'))
        only_estimator = lambda params: {key: value for key, value in params.items()
                                        if not key.startswith('/corridor01_gt_visualizer/')}
        self.assertEqual(only_estimator(off), only_estimator(on))
        self.assertFalse(off['/corridor01_gt_visualizer/enabled'])
        self.assertTrue(on['/corridor01_gt_visualizer/enabled'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
