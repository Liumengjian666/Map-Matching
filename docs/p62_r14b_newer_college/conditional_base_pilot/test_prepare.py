import tempfile
from pathlib import Path
import struct
import unittest
import numpy as np
from prepare import pcd_format, rigid

HERE=Path(__file__).resolve().parent
HEADER=(b'VERSION 0.7\nFIELDS x y z intensity t reflectivity ring noise range\n'
        b'SIZE 4 4 4 4 4 2 1 2 4\nTYPE F F F F U U U U U\n'
        b'COUNT 1 1 1 1 1 1 1 1 1\nWIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA binary\n')


class PrepareTests(unittest.TestCase):
    def probe(self, suffix=b'', header=HEADER, record=None):
        with tempfile.TemporaryDirectory(dir=HERE,prefix='unit_pcd_') as folder:
            p=Path(folder)/'one.pcd'
            p.write_bytes(header+(record if record is not None else struct.pack('<ffffIHBHI',1,2,3,4,5,6,7,8,9))+suffix)
            return pcd_format(p)

    def test_padding_is_not_point(self):
        r=self.probe(b'\0'*3846)
        self.assertEqual(r['points'],1);self.assertEqual(r['finite_xyz'],1)
        self.assertEqual(r['declared_point_bytes'],29);self.assertEqual(r['tail_bytes'],3846)
        self.assertEqual(r['t_min_raw'],5)

    def test_nonzero_tail_rejected(self):
        with self.assertRaises(ValueError): self.probe(b'\0\x01')

    def test_truncated_record_rejected(self):
        with self.assertRaises(ValueError): self.probe(record=b'\0'*28)

    def test_field_type_rejected(self):
        with self.assertRaises(ValueError): self.probe(header=HEADER.replace(b'F F F F U',b'F F F F F'))

    def test_rigid_rejects_reflection_and_millimeter_misuse_not_fitted(self):
        p=np.eye(4);self.assertTrue(rigid(p));p[0,0]=-1;self.assertFalse(rigid(p))
        p=np.eye(4);p[3,3]=0;self.assertFalse(rigid(p))


if __name__=='__main__': unittest.main(verbosity=2)
