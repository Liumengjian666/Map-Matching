#!/usr/bin/env python3
"""Small transform/limiter/algebra tests; never execute localization."""
import numpy as np
from analyze import limited, next_position, lidar_to_imu


def rz(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c,-s,0], [s,c,0], [0,0,1.]])


previous = (np.zeros(3), np.eye(3))
raw = (np.array([.7,0,0]), rz(np.deg2rad(10)))
p, r = limited(raw, previous)
np.testing.assert_allclose(p, [.5,0,0], atol=1e-14)
np.testing.assert_allclose(r, rz(np.deg2rad(5)), atol=1e-14)
extrinsic = np.eye(4)
extrinsic[:3,:3] = rz(np.pi/2)
extrinsic[:3,3] = [.08,.029,.03]
lidar = (np.array([1.,2.,3.]), rz(np.pi/2))
imu = lidar_to_imu(lidar, extrinsic)
np.testing.assert_allclose(imu[1], np.eye(3), atol=1e-14)
np.testing.assert_allclose(imu[0], lidar[0]-extrinsic[:3,3], atol=1e-14)
current = (np.array([1.,0,0]), np.eye(3))
corrected = (np.array([1.05,0,0]), np.eye(3))
np.testing.assert_allclose(next_position(current, previous), [2.,0,0])
np.testing.assert_allclose(next_position(corrected, previous)-next_position(current, previous), [.1,0,0])
np.testing.assert_allclose(.6*(corrected[0]-current[0])/.1, [.3,0,0])
rotated = (np.array([1.,0,0]), rz(np.pi/2))
np.testing.assert_allclose(next_position(rotated, previous), [1.,1.,0], atol=1e-14)
print('PASS frozen limiter, LiDAR-to-IMU lever arm/direction, next-guess and velocity algebra')
