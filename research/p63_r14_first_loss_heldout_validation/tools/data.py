"""Exact binary PCD/PointCloud2 adapter helpers. No pose reference access."""
import hashlib
from pathlib import Path
import numpy as np

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def pcd_xyz(path):
    blob = Path(path).read_bytes()
    end = blob.index(b'DATA binary\n') + len(b'DATA binary\n')
    header = {}
    for line in blob[:end].decode().splitlines():
        if line and not line.startswith('#'):
            key, *values = line.split()
            header[key] = values
    fields, sizes = header['FIELDS'], list(map(int, header['SIZE']))
    counts = list(map(int, header['COUNT']))
    assert all(s == 4 and c == 1 for s, c in zip(sizes, counts))
    formats = [{'F': '<f4', 'U': '<u4', 'I': '<i4'}[t] for t in header['TYPE']]
    dtype = np.dtype(list(zip(fields, formats)))
    count = int(header['POINTS'][0])
    assert count == int(header['WIDTH'][0])*int(header['HEIGHT'][0])
    assert len(blob)-end == count*dtype.itemsize
    points = np.frombuffer(blob, dtype=dtype, count=count, offset=end)
    if 't' in fields:
        assert not np.any(points['t']), 'Expected the certified instantaneous-depth adapter.'
    xyz = np.column_stack([points[k] for k in ('x', 'y', 'z')])
    return xyz, header

def xyzi_bytes(xyz):
    array = np.zeros((len(xyz), 4), dtype='<f4')
    array[:, :3] = xyz
    assert array[:, :3].tobytes() == xyz.tobytes()
    return array.tobytes()

def write_xyzi_pcd(path, xyz):
    count = len(xyz)
    header = ('VERSION .7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\n'
              'COUNT 1 1 1 1\nWIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
              'POINTS %d\nDATA binary\n' % (count, count)).encode()
    with Path(path).open('xb') as stream:
        stream.write(header+xyzi_bytes(xyz))
    after, _ = pcd_xyz(path)
    assert after.tobytes() == xyz.tobytes()

def message(frame):
    import rospy
    from sensor_msgs.msg import PointCloud2, PointField
    assert sha(frame['path']) == frame['sha256']
    xyz, _ = pcd_xyz(frame['path'])
    assert hashlib.sha256(xyz.tobytes()).hexdigest() == frame['xyz_sha256']
    msg = PointCloud2()
    ns = frame['stamp_ns']
    msg.header.stamp = rospy.Time(ns//10**9, ns%10**9)
    msg.header.frame_id = frame['frame_id']
    msg.width, msg.height = len(xyz), 1
    msg.fields = [PointField(k, i*4, PointField.FLOAT32, 1) for i, k in enumerate(('x','y','z','intensity'))]
    msg.is_bigendian = False
    msg.point_step, msg.row_step = 16, len(xyz)*16
    msg.data = xyzi_bytes(xyz)
    msg.is_dense = bool(np.isfinite(xyz).all())
    return msg
