"""No alignment: audit archived artifacts, labels and distinct evaluation poses."""
import csv
import hashlib
import json
from pathlib import Path
import unittest
import numpy as np

DOC = Path(__file__).resolve().parent


def rows(name):
    with (DOC / name).open() as stream:
        return list(csv.DictReader(stream))


class FrozenResultsTest(unittest.TestCase):
    def test_output_byte_identity(self):
        frozen = json.loads((DOC / 'output_freeze.json').read_text())
        for source, entry in frozen['files'].items():
            data = (DOC / Path(source).name).read_bytes()
            self.assertEqual(len(data), entry['bytes'])
            self.assertEqual(hashlib.sha256(data).hexdigest(), entry['sha256'])

    def test_actual_grid_not_getter_only(self):
        grid = rows('grid_audit.csv')
        self.assertEqual(len(grid), 1000)
        for row in grid:
            for key in ('leaf_x', 'leaf_y', 'leaf_z'):
                self.assertEqual(float(row[key]), float(np.float32(.8)))
            self.assertEqual(row['pass'], '1')

    def test_input_parity_and_budgets(self):
        parity = rows('input_parity.csv')
        self.assertEqual(len(parity), 300)
        self.assertTrue(all(r['pass'] == '1' for r in parity))
        align = rows('corrected_alignments.csv')
        self.assertEqual(len({(r['scan_index'], r['arm']) for r in align}), 300)
        self.assertEqual(sum(int(r['align_calls']) for r in align), 300)
        hessian = rows('corrected_hessian.csv')
        self.assertEqual(sum(int(r['jet_calls']) for r in hessian), 600)
        for ref in ('R4_FULL_360_FROZEN', 'R5_FULL_360_COMMON'):
            subset = [r for r in hessian if r['evaluation_pose'] == ref]
            self.assertEqual(len(subset), 300)
            self.assertEqual(len({(r['scan_index'], r['arm']) for r in subset}), 300)

    def test_reported_counts_and_frozen_criterion(self):
        results = json.loads((DOC / 'results.json').read_text())
        for ref in ('R5_GRID08_FIXED', 'R5_GRID08_NEW'):
            classification = results['references'][ref]['frozen_r4_classification']
            self.assertGreaterEqual(classification['comparable_triplets'], 80)
            for arm, stats in results['references'][ref]['groups'].items():
                name = 'R4_FULL_360_FROZEN' if ref.endswith('FIXED') else 'R5_FULL_360_COMMON'
                subset = [r for r in rows('corrected_hessian.csv') if r['arm'] == arm and r['evaluation_pose'] == name]
                labels = [r for r in subset if r['spd'] == '1' and float(r['ratio6']) <= .001
                          and float(r['ratio_translation']) <= .01]
                self.assertEqual(len(labels), stats['weak_labels'])


if __name__ == '__main__':
    unittest.main()
