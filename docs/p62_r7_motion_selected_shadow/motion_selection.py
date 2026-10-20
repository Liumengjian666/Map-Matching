"""Raw-motion-only window ranking. No registration scores, poses, or GT are inputs."""
import numpy as np


def weighted_quantile(values,weights,q):
    values=np.asarray(values,dtype=float);weights=np.asarray(weights,dtype=np.int64)
    if not len(values) or len(values)!=len(weights) or not np.isfinite(values).all() or np.any(weights<=0):
        raise ValueError('Invalid weighted empirical distribution')
    order=np.argsort(values,kind='stable')
    cumulative=np.cumsum(weights[order])
    # Inverse weighted ECDF, without interpolation or equal-sample weighting.
    index=int(np.searchsorted(cumulative,q*int(cumulative[-1]),side='left'))
    return float(values[order[min(index,len(order)-1)]])


def interval_motion(times,gyro,start,end,max_gap=None):
    """ZOH gyro-norm distribution on integer-ns [start,end); both ends bracketed."""
    t=np.asarray(times,dtype=np.int64);g=np.asarray(gyro,dtype=float)
    if end<=start or len(t)<2 or g.shape!=(len(t),3):raise ValueError('Invalid motion inputs')
    first=int(np.searchsorted(t,start,side='right'))-1
    last=int(np.searchsorted(t,end,side='left'))
    if first<0 or last>=len(t):raise ValueError('IMU boundary not bracketed')
    if np.any(np.diff(t[first:last+1])<=0):raise ValueError('Nonmonotonic or duplicate IMU')
    gaps=np.diff(t[first:last+1])
    if max_gap is not None and len(gaps) and int(gaps.max())>max_gap:raise ValueError('IMU gap exceeds explicit caller limit')
    if not np.isfinite(g[first:last+1]).all():raise ValueError('Nonfinite gyro')
    left=np.maximum(t[first:last],start);right=np.minimum(t[first+1:last+1],end)
    weights=right-left
    if not len(weights) or np.any(weights<=0) or int(weights.sum())!=end-start:raise ValueError('Incomplete time weights')
    norms=np.linalg.norm(g[first:last],axis=1)
    return {'weighted_p90_rad_s':weighted_quantile(norms,weights,.9),
            'weighted_median_rad_s':weighted_quantile(norms,weights,.5),
            'weights_ns':int(weights.sum()),'imu_samples':last-first+1,
            'imu_first_ns':int(t[first]),'imu_last_ns':int(t[last]),'max_gap_ns':int(gaps.max())}


def rank_key(row):
    return (-row['weighted_p90_rad_s'],-row['weighted_median_rad_s'],row['start_ns'],row['raw_start_index'])
