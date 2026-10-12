"""No replay or align: check the fixed selection and copied preprocessing."""
import csv
import json
import unittest
from pathlib import Path
import yaml


def function(text,start):
    pos=text.index(start);brace=text.index('{',pos);depth=1;end=brace+1
    while depth:
        depth+=(text[end]=='{')-(text[end]=='}');end+=1
    return text[pos:end]


class ContractTests(unittest.TestCase):
    def setUp(self):
        self.repo=Path(__file__).resolve().parents[4]

    def test_preprocessing_and_hash_are_verbatim(self):
        original=(self.repo/'src/dog_prior_map_localization/src/dog_prior_map_ndt_node.cpp').read_text()
        probe=(Path(__file__).parent/'source_probe.cpp').read_text()
        for name in ('void finalizeCloud(','pcl::PointCloud<pcl::PointXYZ>::Ptr voxelDown(',
                     'pcl::PointCloud<pcl::PointXYZ>::Ptr preprocess(','static uint64_t stableCloudHash('):
            self.assertEqual(function(original,name),function(probe,name))

    def test_probe_parameters_match_frozen_p19(self):
        p=yaml.safe_load((self.repo/'docs/p19_r1_visual_replication_tail_risk/runs/repeat_visual/rosparams.yaml').read_text())
        actual=p['lidar_update']
        for key,value in [('scan_min_range',.5),('scan_max_range',80),('ndt_source_voxel_size',.25),
                          ('ndt_source_voxel_z_size',.25),('ndt_max_source_points',1400)]:
            self.assertEqual(actual[key],value)
        self.assertNotIn('scan_min_z',actual);self.assertNotIn('scan_max_z',actual)
        self.assertEqual(p['deskew']['T_imu_lidar_translation_m'],[.08,.029,.03])
        self.assertTrue(p['imu']['midpoint_interval_input_enable']);self.assertTrue(p['imu']['use_acc_for_position'])

    def test_selected_frames_are_fixed_unique_and_nonvisual(self):
        selection=self.repo/'.p22_r1_fixed_source_ndt_pair_test/frame_selection.csv'
        if not selection.exists():selection=self.repo/'docs/p22_r1_fixed_source_ndt_pair_test/frame_selection.csv'
        with selection.open() as f:r=list(csv.DictReader(f))
        self.assertEqual([int(x['frame_index']) for x in r],[2,284,293,296,308,347,368,396])
        self.assertTrue(all(x['visual_applied']=='False' for x in r))
        for x in r[1:]:self.assertLessEqual(abs(float(x['relative_time_s'])-float(x['target_relative_time_s'])),.15)

    def test_probe_contains_no_align_or_filter_update(self):
        text=(Path(__file__).parent/'source_probe.cpp').read_text()
        self.assertNotIn('.align(',text);self.assertNotIn('processNdtObservationLocked',text)

    def test_measured_stop_does_not_claim_pair_or_gt_results(self):
        location=self.repo/'.p22_r1_fixed_source_ndt_pair_test'
        if not (location/'reconstruction_result.json').exists():
            location=self.repo/'docs/p22_r1_fixed_source_ndt_pair_test'
        receipt=json.loads((location/'reconstruction_result.json').read_text())
        self.assertEqual(receipt['source_matched'],0)
        self.assertEqual(receipt['NDT_align_calls'],0)
        self.assertFalse(receipt['GT_LOADED'])
        self.assertEqual(receipt['next_gate'],'STOP_NO_PAIR_TEST')


if __name__=='__main__':unittest.main()
