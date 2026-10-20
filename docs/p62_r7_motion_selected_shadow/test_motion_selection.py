import unittest
import numpy as np
from motion_selection import interval_motion,weighted_quantile,rank_key


class MotionContracts(unittest.TestCase):
    def test_time_weights_not_sample_counts(self):
        self.assertEqual(weighted_quantile([1,9],[99,1],.9),1)
        self.assertEqual(weighted_quantile([1,9],[1,99],.9),9)

    def test_clipped_integer_boundaries(self):
        r=interval_motion([0,10,20,30],[[1,0,0],[2,0,0],[3,0,0],[100,0,0]],5,25,30)
        self.assertEqual(r['weights_ns'],20);self.assertEqual(r['weighted_p90_rad_s'],3)

    def test_end_sample_not_weighted_past_end(self):
        r=interval_motion([0,10,20],[[1,0,0],[2,0,0],[999,0,0]],0,20,30)
        self.assertEqual(r['weighted_p90_rad_s'],2)

    def test_missing_start_or_end(self):
        for start,end in [(-1,10),(0,21)]:
            with self.assertRaises(ValueError):interval_motion([0,10,20],np.zeros((3,3)),start,end,30)

    def test_gap_rejection(self):
        with self.assertRaises(ValueError):interval_motion([0,10,100],np.zeros((3,3)),0,100,30)

    def test_nonmonotonic_rejection(self):
        with self.assertRaises(ValueError):interval_motion([0,10,10,20],np.zeros((4,3)),0,20,30)

    def test_nonfinite_rejection(self):
        with self.assertRaises(ValueError):interval_motion([0,10,20],[[0,0,0],[float('nan'),0,0],[0,0,0]],0,20,30)

    def test_tie_order(self):
        base={'weighted_p90_rad_s':2,'weighted_median_rad_s':1,'start_ns':10,'raw_start_index':1}
        self.assertLess(rank_key(dict(base,weighted_median_rad_s=2)),rank_key(base))
        self.assertLess(rank_key(base),rank_key(dict(base,start_ns=11)))


if __name__=='__main__':unittest.main()
