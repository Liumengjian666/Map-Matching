"""Bounded artifact/identity tests; does not run NDT, replay, or GT."""
import csv
import json
import unittest
from pathlib import Path


def function(text,start):
    pos=text.index(start);brace=text.index('{',pos);depth=1;end=brace+1
    while depth:
        depth+=(text[end]=='{')-(text[end]=='}');end+=1
    return text[pos:end]


class ContractTests(unittest.TestCase):
    def setUp(self):
        self.repo=Path(__file__).resolve().parents[4]
        self.scripts=Path(__file__).resolve().parent
        self.cache=self.repo/'.p23_r1_budgeted_coupled_subspace_refinement'

    def test_frozen_helpers_are_production_functions(self):
        production=(self.repo/'src/dog_prior_map_localization/src/dog_prior_map_ndt_node.cpp').read_text()
        helper=(self.scripts/'frozen_ndt_rules.hpp').read_text()
        for signature in ('void finalizeCloud(','pcl::PointCloud<pcl::PointXYZ>::Ptr voxelDown(',
                          'static uint64_t stableCloudHash(','bool limitNdtStep('):
            self.assertEqual(function(production,signature),function(helper,signature))

    def test_exact_selected_ids_and_no_gt(self):
        location=self.cache/'input_freeze.json'
        if not location.exists():location=self.repo/'docs/p23_r1_budgeted_coupled_subspace_refinement/input_freeze.json'
        freeze=json.loads(location.read_text())
        self.assertFalse(freeze['GT_LOADED'])
        self.assertEqual([int(r['frame_index']) for r in freeze['selection']],[2,284,293,296,308,347,368,396])
        self.assertEqual(len({r['scan_ns'] for r in freeze['selection']}),8)
        self.assertTrue(all(isinstance(r['scan_ns'],str) and int(r['scan_ns'])>10**18 for r in freeze['selection']))

    def test_capture_default_false_and_single_align_site(self):
        production=(self.repo/'src/dog_prior_map_localization/src/dog_prior_map_ndt_node.cpp').read_text()
        self.assertIn('"output/experimental_ndt_source_capture_enable", false',production)
        self.assertEqual(production.count('ndt_.align('),1)
        self.assertLess(production.index('ndt_.setInputSource(source);'),production.index('experimental_source_capture_.save('))

    def test_actual_eight_sources_pass(self):
        location=self.cache/'capture_gate/source_capture_summary.csv'
        if not location.exists():location=self.repo/'docs/p23_r1_budgeted_coupled_subspace_refinement/source_capture_summary.csv'
        with location.open() as stream:rows=list(csv.DictReader(stream))
        self.assertEqual(len(rows),8)
        self.assertTrue(all(r['status']=='PASS' and r['online_hash']==r['saved_hash'] and r['points']=='1400' for r in rows))


if __name__=='__main__':unittest.main()
