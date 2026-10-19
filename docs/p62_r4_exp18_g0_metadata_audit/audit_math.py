"""Pure metadata arithmetic. No estimation, alignment or trajectory generation."""
from decimal import Decimal, ROUND_HALF_EVEN
import numpy as np


def seconds_to_ns(value):
    seconds=Decimal.from_float(value) if isinstance(value,float) else Decimal(str(value))
    return int((seconds*Decimal(1000000000)).to_integral_value(rounding=ROUND_HALF_EVEN))


def stats(values):
    if not len(values):return {'count':0,'min':'NA','median':'NA','p95':'NA','max':'NA'}
    a=np.asarray(values,dtype=np.float64)
    return {'count':len(values),'min':float(a.min()),'median':float(np.median(a)),
            'p95':float(np.quantile(a,.95,interpolation='linear')),'max':float(a.max())}


def cloud_array(msg):
    mapping={1:'i1',2:'u1',3:'i2',4:'u2',5:'i4',6:'u4',7:'f4',8:'f8'}
    endian='>' if msg.is_bigendian else '<'
    names=[];formats=[];offsets=[]
    for field in msg.fields:
        if field.datatype not in mapping or field.count<1:raise ValueError('INVALID_POINT_FIELD')
        names.append(field.name);offsets.append(field.offset)
        formats.append((endian+mapping[field.datatype],(field.count,)) if field.count>1 else endian+mapping[field.datatype])
    dtype=np.dtype({'names':names,'formats':formats,'offsets':offsets,'itemsize':msg.point_step})
    if msg.row_step<msg.width*msg.point_step or len(msg.data)!=msg.row_step*msg.height:
        raise ValueError('POINTCLOUD_LAYOUT_LENGTH')
    return np.ndarray((msg.height,msg.width),dtype=dtype,buffer=msg.data,
                      strides=(msg.row_step,msg.point_step))


def imu_support(times,start,end):
    """Bracket the ENTIRE point interval; do not silently crop edge scans."""
    if end<start or len(times)<2 or np.any(np.diff(times)<=0):
        return {'status':'INVALID_INTERVAL_OR_IMU_ORDER'}
    first=int(np.searchsorted(times,start,side='right'))-1
    last=int(np.searchsorted(times,end,side='left'))
    if first<0:return {'status':'NO_START_BRACKET'}
    if last>=len(times):return {'status':'NO_END_BRACKET'}
    support=times[first:last+1]
    return {'status':'COVERED','imu_before_start_ns':int(times[first]),
            'imu_at_or_after_end_ns':int(times[last]),'bracket_samples':len(support),
            'max_gap_ns':int(np.max(np.diff(support))) if len(support)>1 else 0}


def quaternion_wxyz_matrix(q):
    q=np.asarray(q,dtype=float)
    if q.shape!=(4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1)>1e-5:
        raise ValueError('INVALID_QUATERNION')
    w,x,y,z=q/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
