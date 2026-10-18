from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
import numpy as np
from check_gt_and_split import ns, read_gt


class GTContractTests(unittest.TestCase):
    def test_decimal_nanoseconds_not_float_rounding(self):
        self.assertEqual(ns('1706671253.339999705'),1706671253339999705)

    def test_zero_quaternion_preserved_not_promoted_to_pose(self):
        with TemporaryDirectory(prefix='p61_r4_test_') as directory:
            p=Path(directory)/'gt.txt';p.write_text('1.0 1 2 3 0 0 0 0\n1.1 2 3 4 0 0 0 0\n')
            _,_,q,_=read_gt(p)
            self.assertTrue(np.all(np.linalg.norm(q,axis=1)==0))

    def test_html_and_duplicate_time_rejected(self):
        with TemporaryDirectory(prefix='p61_r4_test_') as directory:
            p=Path(directory)/'gt.txt';p.write_text('<html>quota exceeded</html>')
            with self.assertRaises(ValueError):read_gt(p)
            p.write_text('1.0 1 2 3 0 0 0 0\n1.0 2 3 4 0 0 0 0\n')
            with self.assertRaises(ValueError):read_gt(p)


if __name__=='__main__':unittest.main()
