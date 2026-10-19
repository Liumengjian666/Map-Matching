import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from audit_math import seconds_to_ns,cloud_array,imu_support,quaternion_wxyz_matrix


class MetadataMathTests(unittest.TestCase):
    def test_decimal_integer_time(self):
        self.assertEqual(seconds_to_ns('1649856227.123456789'),1649856227123456789)
    def test_double_conversion_not_float_multiplication(self):
        value=1649856227.1234567
        from decimal import Decimal
        self.assertEqual(seconds_to_ns(value),int((Decimal.from_float(value)*1000000000).to_integral_value()))
    def test_complete_imu_bracket(self):
        out=imu_support(np.array([10,20,30,40]),12,33)
        self.assertEqual((out['imu_before_start_ns'],out['imu_at_or_after_end_ns']),(10,40))
    def test_missing_start_is_not_cropped(self):
        self.assertEqual(imu_support(np.array([20,30,40]),12,33)['status'],'NO_START_BRACKET')
    def test_missing_end_is_not_extrapolated(self):
        self.assertEqual(imu_support(np.array([10,20,30]),12,33)['status'],'NO_END_BRACKET')
    def test_reversed_imu_is_not_sorted(self):
        self.assertEqual(imu_support(np.array([10,30,20]),12,25)['status'],'INVALID_INTERVAL_OR_IMU_ORDER')
    def test_exact_bracket(self):
        out=imu_support(np.array([10,20,30]),10,30)
        self.assertEqual(out['status'],'COVERED');self.assertEqual(out['max_gap_ns'],10)
    def test_organized_pointcloud_row_padding(self):
        data=bytearray(48)
        np.ndarray((2,2),dtype='<f8',buffer=data,strides=(24,8))[:]=[[1,2],[3,4]]
        msg=NS(fields=[NS(name='timestamp',datatype=8,count=1,offset=0)],
            point_step=8,row_step=24,height=2,width=2,is_bigendian=False,data=bytes(data))
        np.testing.assert_array_equal(cloud_array(msg)['timestamp'],[[1,2],[3,4]])
    def test_bad_pointcloud_layout(self):
        msg=NS(fields=[NS(name='x',datatype=7,count=1,offset=0)],point_step=4,
            row_step=4,height=1,width=1,is_bigendian=False,data=b'')
        with self.assertRaises(ValueError):cloud_array(msg)
    def test_quaternion_order_and_roundtrip(self):
        r=quaternion_wxyz_matrix([np.cos(.4),0,0,np.sin(.4)])
        np.testing.assert_allclose(r.T@r,np.eye(3),atol=1e-14)
        self.assertAlmostEqual(np.linalg.det(r),1)
        np.testing.assert_allclose(r@np.array([1,0,0]),[np.cos(.8),np.sin(.8),0],atol=1e-14)
    def test_reject_nonunit_quaternion(self):
        with self.assertRaises(ValueError):quaternion_wxyz_matrix([2,0,0,0])


if __name__=='__main__':unittest.main()
