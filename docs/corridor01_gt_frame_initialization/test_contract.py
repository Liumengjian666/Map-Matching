#!/usr/bin/env python3
"""Focused guards only: never run a valid NDT optimization or a ROS replay."""
import argparse
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np

HERE = Path(__file__).resolve().parent
WORKSPACE = HERE.parents[1]
sys.path.insert(0,str(WORKSPACE/'src/dog_prior_map_localization/scripts/p32_gt_frame'))
from prepare import at, load_gt
SPEC = importlib.util.spec_from_file_location('prep',HERE/'prepare_geometry.py')
PREP = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PREP)
INPUT = BINARY = None


class ContractTest(unittest.TestCase):
    def test_first_IMU_covered_scan_has_no_legal_GT_pose(self):
        selection = json.loads((INPUT/'selection.json').read_text())
        rows = load_gt(PREP.ROOT/'gt/corridor01_gt.txt')
        with self.assertRaisesRegex(ValueError,'NO_GT_EXTRAPOLATION'):
            at(rows,selection['first_full_IMU_scan']['scan_ns'])
        _, bracket = at(rows,selection['selected_first_full_IMU_and_GT_scan']['scan_ns'])
        self.assertEqual(bracket['lower_ns'],bracket['upper_ns'])

    def test_matrix_order_and_missing_transforms_not_fabricated(self):
        r = json.loads((INPUT/'hypothesis_freeze.json').read_text())
        T = {k:np.array(v['values']) for k,v in r['matrices'].items()}
        np.testing.assert_allclose(T['H1_conditional_T_N_L'],T['T_N_M']@T['T_W_GT'],atol=1e-13)
        np.testing.assert_allclose(T['H2_conditional_T_N_L'],T['T_N_M']@T['T_W_GT']@T['T_I_L_frozen_proper'],atol=1e-13)
        self.assertIsNone(r['T_M_W_certified'])
        self.assertTrue(r['H3_status'].startswith('NOT_CONSTRUCTED'))
        self.assertTrue(r['H4_status'].startswith('NOT_CONSTRUCTED'))

    def test_official_matrix_not_silently_treated_as_proper(self):
        r = json.loads((INPUT/'hypothesis_freeze.json').read_text())['matrices']
        self.assertFalse(r['T_I_L_original']['rigid'])
        self.assertTrue(r['T_I_L_frozen_proper']['rigid'])
        bad = np.eye(4);bad[0,0] = -1
        self.assertFalse(PREP.certificate(bad)['rigid'])

    def test_probe_rejects_bad_initial_matrices_before_any_map_or_align(self):
        for kind in ['reflection','nonfinite','truncated','extra']:
            with self.subTest(kind=kind), tempfile.TemporaryDirectory(dir=INPUT.parent) as tmp:
                root = Path(tmp)
                T = np.array([np.eye(4),np.eye(4)],dtype='<f8')
                if kind == 'reflection':T[0,0,0] = -1
                if kind == 'nonfinite':T[0,0,3] = np.nan
                payload = T.tobytes()
                if kind == 'truncated':payload = payload[:-1]
                if kind == 'extra':payload += b'x'
                with (root/'hypotheses.f64').open('xb') as stream:stream.write(payload)
                result = subprocess.run([str(BINARY),str(root),'MUST_NOT_READ_MAP',str(root),'--one-H2-ndt'],capture_output=True,text=True,timeout=5)
                self.assertNotEqual(result.returncode,0)
                self.assertFalse((root/'ndt.csv').exists())
                self.assertFalse((root/'geometry.csv').exists())


if __name__ == '__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--input',type=Path,required=True)
    parser.add_argument('--binary',type=Path,required=True)
    args=parser.parse_args()
    INPUT=args.input.resolve();BINARY=args.binary.resolve()
    unittest.main(argv=[sys.argv[0]],verbosity=2)
