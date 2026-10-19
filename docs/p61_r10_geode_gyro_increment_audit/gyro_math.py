"""Pure bounded SO(3) diagnostics. No ROS, estimator, pose search or I/O."""
import bisect
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.linalg import logm


def check_rotation(R):
    R=np.asarray(R,dtype=float)
    if R.shape!=(3,3) or not np.isfinite(R).all():raise ValueError('ROTATION_NONFINITE_OR_SHAPE')
    if np.linalg.norm(R.T@R-np.eye(3))>1e-8 or abs(np.linalg.det(R)-1)>1e-8:
        raise ValueError('ROTATION_NOT_SO3')
    return R


def angle_deg(R):
    R=check_rotation(R)
    # Check the trace-domain bound explicitly; atan2 quaternion log is stable
    # at both zero and pi, unlike arccos(trace) used alone.
    c=(np.trace(R)-1)/2
    if c < -1-1e-8 or c > 1+1e-8:raise ValueError('TRACE_INVALID')
    c=float(np.clip(c,-1,1))
    q=Rotation.from_matrix(R).as_quat()
    if q[3]<0:q=-q
    return float(np.rad2deg(2*np.arctan2(np.linalg.norm(q[:3]),q[3])))


def independent_angle_deg(R):
    R=check_rotation(R);L=logm(R)
    # The matrix-log principal branch at exact pi can be complex even for a
    # legitimate real rotation. Its Frobenius norm still gives sqrt(2)*pi.
    if np.max(np.abs(L.imag))>1e-7 and abs((np.trace(R)-1)/2+1)>1e-8:
        raise ValueError('INDEPENDENT_LOGM_NONREAL_AWAY_FROM_PI')
    return float(np.rad2deg(np.linalg.norm(L,'fro')/np.sqrt(2)))


def samples_from_records(records):
    if any(type(x['stamp_ns']) is not int for x in records):raise ValueError('NONINTEGER_SAMPLE_STAMP')
    return [(x['stamp_ns'],x['gyro_radps']) for x in records]


def imu_relative_rotation(RML_previous,RML_current,RIL):
    check_rotation(RIL)
    # Right division implements R_M_I = R_M_L * inverse(R_I_L),
    # retaining frozen coefficients rather than editing the extrinsic.
    prev=np.linalg.solve(RIL.T,check_rotation(RML_previous).T).T
    curr=np.linalg.solve(RIL.T,check_rotation(RML_current).T).T
    check_rotation(prev);check_rotation(curr)
    return check_rotation(prev.T@curr)


def integrate_zoh(samples,start_ns,end_ns,bias,median_dt_ns):
    """samples=(integer ns, body gyro rad/s), causal head plus samples<=end.

    Fail closed; callers retain invalid intervals instead of manufacturing a
    rotation. No sorting, interpolation or future sample is permitted.
    """
    if not isinstance(start_ns,(int,np.integer)) or not isinstance(end_ns,(int,np.integer)):
        raise ValueError('NONINTEGER_ENDPOINT')
    if end_ns<=start_ns or not np.isfinite(median_dt_ns) or median_dt_ns<=0:
        raise ValueError('INVALID_INTERVAL_OR_MEDIAN')
    if not samples:raise ValueError('NO_IMU')
    stamps=[s[0] for s in samples]
    if any(not isinstance(t,(int,np.integer)) for t in stamps):raise ValueError('NONINTEGER_STAMP')
    if any(a>=b for a,b in zip(stamps,stamps[1:])):raise ValueError('NONMONOTONIC_OR_DUPLICATE')
    if stamps[0]>start_ns:raise ValueError('NO_CAUSAL_HEAD')
    if stamps[-1]>end_ns:raise ValueError('FUTURE_SAMPLE_FORBIDDEN')
    if bisect.bisect_right(stamps,start_ns)!=1:raise ValueError('NOT_NEAREST_CAUSAL_HEAD_SLICE')
    bias=np.asarray(bias,dtype=float)
    if bias.shape!=(3,) or not np.isfinite(bias).all():raise ValueError('INVALID_BIAS')
    gyro=np.asarray([s[1] for s in samples],dtype=float)
    if gyro.shape!=(len(samples),3) or not np.isfinite(gyro).all():raise ValueError('NONFINITE_GYRO')
    gaps=[b-a for a,b in zip(stamps,stamps[1:])]+[start_ns-stamps[0],end_ns-stamps[-1]]
    maximum=max(gaps)
    if maximum>3*median_dt_ns:raise ValueError('MAX_GAP_EXCEEDED')
    R=np.eye(3);segments=[]
    for j,(stamp,w) in enumerate(zip(stamps,gyro)):
        left=max(start_ns,stamp);right=min(end_ns,stamps[j+1] if j+1<len(stamps) else end_ns)
        if right<=left:continue # A sample exactly at the end has zero hold.
        dt=(right-left)*1e-9
        R=R@Rotation.from_rotvec((w-bias)*dt).as_matrix()
        segments.append(dict(sample_stamp_ns=int(stamp),start_ns=int(left),end_ns=int(right),dt_ns=int(right-left)))
    if not segments or sum(s['dt_ns'] for s in segments)!=end_ns-start_ns:raise ValueError('INCOMPLETE_COVERAGE')
    check_rotation(R)
    return R,dict(sample_count=len(samples),positive_hold_count=len(segments),first_sample_ns=int(stamps[0]),
        last_sample_ns=int(stamps[-1]),start_head_age_ns=int(start_ns-stamps[0]),end_hold_age_ns=int(end_ns-stamps[-1]),
        max_gap_ns=int(maximum),segments=segments,future_sample_used=False)


def classify(residuals,valid):
    if len(residuals)!=11 or len(valid)!=11 or not all(valid):return 'NOT_EVALUABLE'
    v=np.asarray(residuals,dtype=float)
    if not np.isfinite(v).all() or np.any(v<0):return 'NOT_EVALUABLE'
    if np.median(v)<=2 and np.max(v)<=5:return 'LIMITED_ANGULAR_CONSISTENCY_SUPPORTED'
    if any(a>5 and b>5 for a,b in zip(v,v[1:])):return 'ANGULAR_CONSISTENCY_CONTRADICTED'
    return 'MIXED_OR_INCONCLUSIVE'
