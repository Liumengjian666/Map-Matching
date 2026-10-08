#!/usr/bin/env python3
"""Synthetic bags only; does not read or re-extract the Corridor01 raw bag."""
import argparse
from pathlib import Path
import struct
import subprocess
import tempfile

import genpy
import rosbag
from sensor_msgs.msg import Imu
from velodyne_msgs.msg import VelodynePacket, VelodyneScan


def fixture(path, with_scan=True, with_imu=True, mode=0x37):
    stamp = 1517157219000000000
    with rosbag.Bag(str(path), 'w') as b:
        if with_scan:
            scan = VelodyneScan()
            scan.header.stamp = genpy.Time(stamp // 10**9, stamp % 10**9)
            scan.header.frame_id = 'cmu_rc2_velodyne'
            packet = VelodynePacket()
            packet.stamp = scan.header.stamp
            data = bytearray(1206)
            for block in range(12):
                struct.pack_into('<HH', data, block*100, 0xeeff, block*10)
                for j in range(32):
                    struct.pack_into('<HB', data, block*100+4+j*3, 5000, 30)
            struct.pack_into('<I', data, 1200, (stamp // 1000) % (3600 * 10**6))
            data[1204], data[1205] = mode, 0x22
            packet.data = bytes(data)
            scan.packets = [packet]
            b.write('/velodyne_packets', scan, t=genpy.Time(1700000000))
        if with_imu:
            m = Imu()
            m.header.frame_id = 'epson'
            m.header.stamp = genpy.Time(stamp // 10**9, stamp % 10**9)
            m.linear_acceleration.z = 9.809
            b.write('/imu/data', m, t=genpy.Time(1700000000, 1))


def main():
    p = argparse.ArgumentParser()
    p.add_argument('binary')
    p.add_argument('calibration')
    args = p.parse_args()
    with tempfile.TemporaryDirectory(prefix='p9_raw_synthetic_') as tmp:
        root = Path(tmp)
        for name, kwargs, expected in [
            ('valid', {}, True), ('no_imu', {'with_imu': False}, False),
            ('no_scan', {'with_scan': False}, False),
            ('dual_return', {'mode': 0x39}, False),
        ]:
            bag, out = root / (name + '.bag'), root / name
            fixture(bag, **kwargs)
            run = subprocess.run([args.binary, str(bag), args.calibration, str(out)], capture_output=True)
            assert (run.returncode == 0) == expected, (name, run.stderr)
            assert (out / 'EXTRACTION_COMPLETE').exists() == expected
            if expected:
                assert (out / 'raw_timed_points.bin').stat().st_size == 384 * 16
                before = (out / 'raw_timed_points.bin').read_bytes()
                duplicate = subprocess.run([args.binary, str(bag), args.calibration, str(out)], capture_output=True)
                assert duplicate.returncode != 0
                assert (out / 'raw_timed_points.bin').read_bytes() == before
    print('SYNTHETIC_BAG_BOUNDARY_TESTS=PASS cases=5')


if __name__ == '__main__':
    main()
