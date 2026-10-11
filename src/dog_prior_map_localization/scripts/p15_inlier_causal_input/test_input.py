#!/usr/bin/env python3
"""Small causal/time/direction/invalid-return regressions; no algorithm tests."""
import numpy as np
from prepare import accumulate, author_submap, clean, matrix

build = author_submap()
poses = [matrix([i, 0, 0], [0, 0, 0, 1]) for i in range(10)]
scans = [dict(stamp_ns=i*100_000_000, end_ns=i*100_000_000+90_000_000,
              points=clean(np.array([[1., 2., 3.], [0., 0., 0.], [np.nan, 0., 0.]], dtype=np.float32)))
         for i in range(10)]
for n in (5, 10):
    cloud, _ = accumulate(scans, poses, n, build)
    assert len(cloud) == n
    expected = np.array([[1+i-9, 2, 3] for i in range(9, 9-n, -1)], dtype=np.float32)
    np.testing.assert_array_equal(cloud, expected)
rotated = [matrix([i*.1, 1, -.2], [0, 0, np.sin(i*.03), np.cos(i*.03)]) for i in range(10)]
world = np.array([3., 4., 5.])
rot_scans = [dict(s, points=((world-t[:3,3]) @ t[:3,:3]).reshape(1,3).astype(np.float32))
             for s, t in zip(scans, rotated)]
cloud, _ = accumulate(rot_scans, rotated, 10, build)
expected = (world-rotated[-1][:3,3]) @ rotated[-1][:3,:3]
np.testing.assert_allclose(cloud, np.tile(expected, (10,1)), atol=2e-6)
for mutation, expected in [('empty', 'EMPTY_MEMBER_SCAN'), ('gap', 'NONCONTIGUOUS_SCANS'), ('future', 'FUTURE_SENSOR_POINT')]:
    altered = [dict(s) for s in scans]
    if mutation == 'empty': altered[8]['points'] = np.empty((0, 3))
    if mutation == 'gap': altered[8]['stamp_ns'] -= 200_000_000
    if mutation == 'future': altered[8]['end_ns'] = altered[9]['end_ns']+1
    try:
        accumulate(altered, poses, 5, build)
        raise AssertionError('Missing rejection '+mutation)
    except ValueError as e:
        assert str(e) == expected
print('PASS N5/N10 current-anchor direction, no future, empty-member and gap rejection, raw-zero guard')
