import unittest
from analyze_native import qualified, runs


def row(index, ratio='.0007', tr='.3', valid='1', direction=(1, 0, 0)):
    return {'scan_index': str(index), 'stamp_ns': str(1000000000 * index),
            'comparable': valid, 'spd': '1', 'ratio6': ratio, 'ratio_translation': tr,
            **{f'translation_weak{i}': str(v) for i, v in enumerate(direction)}}


class NativeStatisticsTest(unittest.TestCase):
    def test_low_six_dof_ratio_alone_not_severe(self):
        self.assertFalse(qualified(row(1)))
        self.assertTrue(qualified(row(1, tr='.005')))
        self.assertFalse(qualified(row(1, tr='.005', valid='0')))

    def test_sparse_run_does_not_claim_all_scans_measured(self):
        result = runs([row(i, tr='.005') for i in (1, 50, 100)])
        self.assertEqual(len(result), 1)
        self.assertFalse(result[0]['all_intermediate_raw_indices_measured'])

    def test_invalid_or_direction_change_breaks_run(self):
        self.assertEqual(runs([row(1, tr='.005'), row(2, tr='.005', valid='0'), row(3, tr='.005')]), [])
        self.assertEqual(runs([row(1, tr='.005'), row(2, tr='.005', direction=(0, 1, 0)), row(3, tr='.005')]), [])

    def test_contiguous_run_is_limited_to_measured_indices(self):
        result = runs([row(i, tr='.005') for i in (2, 3, 4)])
        self.assertTrue(result[0]['all_intermediate_raw_indices_measured'])
        self.assertEqual((result[0]['start_index'], result[0]['end_index']), (2, 4))


if __name__ == '__main__': unittest.main()
