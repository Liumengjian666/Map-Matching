import csv
import hashlib
import json
from pathlib import Path
import unittest

DOC = Path(__file__).resolve().parent


class FrozenArtifactsTest(unittest.TestCase):
    def test_archived_raw_outputs_byte_identity(self):
        for record in json.loads((DOC / 'artifact_manifest.json').read_text()):
            data = (DOC / record['archive']).read_bytes()
            self.assertEqual(len(data), record['bytes'])
            self.assertEqual(hashlib.sha256(data).hexdigest(), record['sha256'])

    def test_real_data_and_call_counts(self):
        result = json.loads((DOC / 'results.json').read_text())
        self.assertTrue(result['bag_identity']['equal'])
        self.assertEqual(result['bag_integrity']['total_messages'], 180523)
        self.assertEqual(result['bag_integrity']['lidar_scans'], 5427)
        self.assertEqual(result['total_align_calls'], 120)
        self.assertEqual(result['total_native_jet_calls'], 120)
        self.assertEqual(result['actual_grid_audits'], 360)
        self.assertFalse(result['fused_smoke_completed'])
        self.assertFalse(result['gt_used'])

    def test_component_and_complete_criterion_are_not_confused(self):
        result = json.loads((DOC / 'results.json').read_text())['phases']['screen']
        self.assertEqual(result['low_6d_component_only_count'], 94)
        self.assertEqual(result['severe_weak_labels'], 0)
        self.assertEqual(result['weak6_translation_dominant_xyz_count'], [1, 68, 31])
        self.assertEqual(result['translation_block_weak_dominant_xyz_count'], [99, 0, 1])


if __name__ == '__main__': unittest.main()
