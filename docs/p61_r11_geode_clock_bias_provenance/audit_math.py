"""Pure timestamp-format and descriptive-record statistics, not synchronization."""
import struct
from decimal import Decimal
import numpy as np


def ros_header(payload):
    if len(payload)<16:raise ValueError('TRUNCATED_HEADER')
    seq,sec,nsec,n=struct.unpack_from('<4I',payload)
    if nsec>=10**9 or 16+n>len(payload):raise ValueError('INVALID_ROS_HEADER')
    return dict(sequence=seq,header_ns=sec*10**9+nsec,frame_id=payload[16:16+n].decode('utf8'),body_offset=16+n)


def lag_ns(header_ns,record_ns):
    if type(header_ns) is not int or type(record_ns) is not int:raise ValueError('INTEGER_NS_REQUIRED')
    return header_ns-record_ns


def statistics(values):
    if not values:return dict(count=0,min_ns='NA',median_ns='NA',p95_linear_ns='NA',max_ns='NA')
    v=np.array(values,dtype=np.int64)
    return dict(count=len(values),min_ns=int(v.min()),median_ns=float(np.median(v)),
        p95_linear_ns=float(np.percentile(v,95)),max_ns=int(v.max()))


def native_seconds(ns):return float(Decimal(ns)/Decimal(10**9))
