import importlib.util
from pathlib import Path
import unittest

path=Path(__file__).resolve().parents[1]/'preflight.py'
spec=importlib.util.spec_from_file_location('r14_preflight',path);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)

class AdmissionTests(unittest.TestCase):
    def test_unknown_size_blocks(self):self.assertEqual(module.admission(10**12,None,0,0),'BLOCKED_PEAK_SIZE_NOT_CERTIFIED')
    def test_unknown_unpack_blocks(self):self.assertEqual(module.admission(10**12,100,None,0),'BLOCKED_PEAK_SIZE_NOT_CERTIFIED')
    def test_unknown_map_blocks(self):self.assertEqual(module.admission(10**12,100,0,None),'BLOCKED_PEAK_SIZE_NOT_CERTIFIED')
    def test_exact_reserve_pass(self):self.assertEqual(module.admission(module.RESERVE+600,100,200,300),'PASS')
    def test_below_reserve_blocks(self):self.assertEqual(module.admission(module.RESERVE+599,100,200,300),'BLOCKED_STORAGE')
    def test_rounded_1_5GB_not_fit(self):self.assertEqual(module.admission(22947463168,1500000000,0,0),'BLOCKED_STORAGE')

if __name__=='__main__':unittest.main()
