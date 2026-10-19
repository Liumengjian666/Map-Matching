import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from acquire_ranges import require_range


class RangeTests(unittest.TestCase):
    def test_exact_range(self):
        require_range(206,'bytes 10-19/100',10,20,100)

    def test_ignored_range_rejected(self):
        with self.assertRaises(RuntimeError):require_range(200,None,10,20,100)

    def test_wrong_start_rejected(self):
        with self.assertRaises(RuntimeError):require_range(206,'bytes 0-19/100',10,20,100)

    def test_wrong_object_length_rejected(self):
        with self.assertRaises(RuntimeError):require_range(206,'bytes 10-19/101',10,20,100)


if __name__=='__main__':unittest.main()
