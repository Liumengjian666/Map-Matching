#!/usr/bin/env python3
"""No bag/map/NN access: test integer-time conversion and deterministic selection."""
import importlib.util
from pathlib import Path
import unittest
import numpy as np

HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('p33_prepare',HERE/'prepare.py')
prep=importlib.util.module_from_spec(spec);spec.loader.exec_module(prep)


class PreparationTest(unittest.TestCase):
    def test_unique_nearest_and_earlier_tie(self):
        scans=[dict(scan_start_ns='100'),dict(scan_start_ns='200'),dict(scan_start_ns='300')]
        self.assertEqual(prep.nearest(scans,150)[0],1)
        self.assertEqual(prep.nearest(scans,210)[0],2)

    def test_duplicate_and_unordered_headers_rejected(self):
        prep.validate_scan_order([dict(scan_start_ns='100'),dict(scan_start_ns='200')])
        for values in [[],['100','100'],['200','100']]:
            with self.assertRaisesRegex(ValueError,'SCAN_HEADER_NOT_STRICTLY_INCREASING'):
                prep.validate_scan_order([dict(scan_start_ns=value) for value in values])

    def test_original_cache_time_not_float_seconds(self):
        raw=np.zeros(1,dtype=prep.RAW_POINT_DTYPE)
        raw[0]=(1.,2.,3.,10000000)
        cloud,_,_=prep.make_cloud(dict(transaction_id='1',cloud_byte_offset='0',cloud_point_count='1',
            scan_start_ns='1517157219000000000',scan_end_ns='1517157219100000000'),raw)
        converted=np.frombuffer(cloud.data,dtype=prep.RAW_DTYPE)
        self.assertEqual(float(converted['time'][0]),float(np.float32(.01)))
        self.assertNotEqual(raw.tobytes(),cloud.data)


if __name__=='__main__':unittest.main(verbosity=2)
