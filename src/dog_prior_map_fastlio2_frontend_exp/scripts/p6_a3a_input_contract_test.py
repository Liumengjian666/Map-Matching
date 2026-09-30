#!/usr/bin/env python3
import csv
import importlib.util
import struct
import tempfile
import unittest
from unittest.mock import patch
from pathlib import Path

SCRIPTS=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('exporter',SCRIPTS/'p6_a3a_export_corridor01_raw_timed.py')
exporter=importlib.util.module_from_spec(spec); spec.loader.exec_module(exporter)

class InputContract(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory(prefix='p6_a3a_contract_')
        self.root=Path(self.temp.name)
        self.points=self.root/'raw_timed_points.bin'
        self.points.write_bytes(b''.join(struct.pack('<ddddQ',1,2,3,4,t) for t in (100,103,119)))
        (self.root/'raw_timed_catalog.csv').write_text(
            'transaction_id,scan_start_ns,scan_end_ns,byte_offset,point_count,provenance\n1,100,120,0,3,RAW_TIMED_SENSOR\n')
        (self.root/'filter_scans.csv').write_text('transaction_id,stamp_ns\n1,120\n')
        self.manifest={'provenance':'RAW_TIMED_SENSOR','GT_USED':'false','LEGACY_STATE_USED':'false',
            'WINDOW_STATE_USED':'false','DESKEW_PERFORMED':'false','scan_count':'1','point_count':'3',
            'first_scan_start_ns':'100','last_scan_end_ns':'120'}
        self.manifest.update({'dataset':'SuperLoc Corridor01','original_bag_absolute_path':str(exporter.BAG),
            'original_bag_sha256':exporter.BAG_SHA,'raw_lidar_topic':'/velodyne_packets',
            'raw_lidar_message_type':'velodyne_msgs/VelodyneScan','sensor_frame_id':'cmu_rc2_velodyne',
            'calibration_sha256':exporter.CAL_SHA,'driver_git_sha':exporter.VELODYNE_SHA})
        self.manifest.update(exporter.TIME_CONTRACT)
        self.manifest['calibration_path']='SYNTHETIC_TEST_NOT_SENSOR_EVIDENCE'
        self.manifest['export_script_git_sha']='0'*40
        for key in ('export_script_sha256','driver_rawdata_sha256','driver_library_sha256',
                    'decoder_source_sha256','decoder_binary_sha256'):
            self.manifest[key]='0'*64
        for stem in ('raw_timed_points','raw_timed_catalog','filter_scans'):
            self.manifest[stem+'_file']=stem+('.bin' if stem=='raw_timed_points' else '.csv')
        self.save_manifest()
    def tearDown(self): self.temp.cleanup()
    def save_manifest(self):
        for stem in ('raw_timed_points','raw_timed_catalog','filter_scans'):
            self.manifest[stem+'_sha256']=exporter.sha256(self.root/self.manifest[stem+'_file'])
        (self.root/'RAW_TIMED_INPUT_MANIFEST.txt').write_text(''.join(f'{k}={v}\n' for k,v in self.manifest.items()))
    def test_nonuniform_timestamps(self):
        self.assertEqual(exporter.validate_export(self.root)['point_count'],3)
        self.assertEqual([r[-1] for r in struct.iter_unpack('<ddddQ',self.points.read_bytes())],[100,103,119])
    def test_manifest_sha(self):
        self.points.write_bytes(self.points.read_bytes()+b'x')
        with self.assertRaisesRegex(ValueError,'SHA_mismatch'): exporter.validate_export(self.root)
    def test_incomplete_sensor_identity(self):
        del self.manifest['driver_git_sha']; self.save_manifest()
        with self.assertRaisesRegex(ValueError,'incomplete_sensor_manifest'): exporter.validate_export(self.root)
    def test_wrong_sensor_identity(self):
        self.manifest['original_bag_sha256']='0'*64; self.save_manifest()
        with self.assertRaisesRegex(ValueError,'provenance_identity_mismatch'): exporter.validate_export(self.root)
    def test_truncation(self):
        self.points.write_bytes(self.points.read_bytes()[:-1]); self.save_manifest()
        with self.assertRaisesRegex(ValueError,'truncated'): exporter.validate_export(self.root)
    def test_zero_timestamp(self):
        self.points.write_bytes(struct.pack('<ddddQ',1,2,3,4,0)*3); self.save_manifest()
        with self.assertRaisesRegex(ValueError,'zero_point'): exporter.validate_export(self.root)
    def test_outside_interval(self):
        self.points.write_bytes(struct.pack('<ddddQ',1,2,3,4,121)*3); self.save_manifest()
        with self.assertRaisesRegex(ValueError,'outside_scan'): exporter.validate_export(self.root)
    def test_legacy_provenance(self):
        self.manifest['provenance']='LEGACY_STATE_DERIVED_SE3_DESKEW'; self.save_manifest()
        with self.assertRaisesRegex(ValueError,'not_RAW'): exporter.validate_export(self.root)
    def test_schedule_cannot_snap(self):
        (self.root/'filter_scans.csv').write_text('transaction_id,stamp_ns\n1,119\n'); self.save_manifest()
        with self.assertRaisesRegex(ValueError,'schedule_timestamp'): exporter.validate_export(self.root)
    def test_time_semantics_cannot_be_renamed(self):
        self.manifest['point_time_unit']='seconds'; self.save_manifest()
        with self.assertRaisesRegex(ValueError,'time_semantics'): exporter.validate_export(self.root)
    def test_hardware_clock_not_claimed(self):
        self.manifest['hardware_clock_or_sync_accuracy_proven']='true'; self.save_manifest()
        with self.assertRaisesRegex(ValueError,'time_semantics'): exporter.validate_export(self.root)
    def test_manifest_counts_and_bounds(self):
        for key,value in [('point_count','4'),('last_scan_end_ns','121')]:
            with self.subTest(key=key):
                previous=self.manifest[key]; self.manifest[key]=value; self.save_manifest()
                with self.assertRaisesRegex(ValueError,'manifest_(counts|time_bounds)_mismatch'):
                    exporter.validate_export(self.root)
                self.manifest[key]=previous
    def test_malformed_hash(self):
        self.manifest['decoder_binary_sha256']='not_a_hash'; self.save_manifest()
        with self.assertRaisesRegex(ValueError,'invalid_identity_hash'): exporter.validate_export(self.root)
    def test_loader_interposition_removed(self):
        with patch.dict(exporter.os.environ,{'LD_PRELOAD':'bad.so','LD_AUDIT':'bad_audit.so',
            'LD_LIBRARY_PATH':'old_decoder','GLIBC_TUNABLES':'arbitrary','ROS_DISTRO':'noetic'}):
            environment=exporter.decoder_environment()
        self.assertFalse(any(key.startswith('LD_') for key in environment))
        self.assertNotIn('GLIBC_TUNABLES',environment)
        self.assertEqual(environment['ROS_DISTRO'],'noetic')
    def test_production_has_no_dense_oracle(self):
        source=(SCRIPTS.parent/'src/fixed_lag_window.cpp').read_text()
        production=source.split('bool FixedLagWindow::latestMarginalCovariance(')[1].split(
            'bool FixedLagWindow::latestMarginalCovarianceDenseReferenceForTest')[0]
        self.assertIn('blockLinearizedSystem',production)
        self.assertIn('solveLatestMarginalColumnsSparse',production)
        for forbidden in ('linearize(','.dense(','DenseReference','inverse('): self.assertNotIn(forbidden,production)

if __name__=='__main__': unittest.main()
