"""Synthetic algebra checks only. Not a registration or localization module."""
import numpy as np


def lower_envelope(values):
    """min w·values: sum(w)=1, 0<=w<=2/B. No IID/probability claim."""
    x=np.asarray(values,dtype=float)
    if x.ndim!=1 or len(x)<2 or not np.isfinite(x).all():raise ValueError('INVALID_BLOCK_VALUES')
    remain=1.;value=0.;cap=2./len(x)
    for v in np.sort(x):
        weight=min(cap,remain);value+=weight*v;remain-=weight
        if remain<=1e-14:break
    return float(value)


def pairwise_advantage(loss_a,loss_b):
    """Positive means a improves over b for every allowed block weighting."""
    a=np.asarray(loss_a,dtype=float);b=np.asarray(loss_b,dtype=float)
    if a.shape!=b.shape:raise ValueError('DIFFERENT_BLOCK_SET')
    return lower_envelope(b-a)


def schur_step(h,g,weak_dim,damping=0.):
    """Solve only an SPD damped quadratic; solves, never explicit inverses."""
    h=np.asarray(h,dtype=float);g=np.asarray(g,dtype=float)
    if h.shape!=(6,6) or g.shape!=(6,) or not 0<weak_dim<6:raise ValueError('DIMENSION')
    if not np.isfinite(h).all() or not np.isfinite(g).all() or not np.allclose(h,h.T,atol=1e-12,rtol=0):raise ValueError('INVALID_QUADRATIC')
    if not np.isfinite(damping) or damping<0:raise ValueError('INVALID_DAMPING')
    h=(h+h.T)/2+damping*np.eye(6);d=weak_dim
    a,b,c=h[:d,:d],h[:d,d:],h[d:,d:]
    np.linalg.cholesky(c)
    if np.linalg.cond(c)>1e8:raise ValueError('STRONG_CONDITION')
    cb=np.linalg.solve(c,b.T);cg=np.linalg.solve(c,g[d:])
    effective=a-b@cb;np.linalg.cholesky(effective)
    if np.linalg.cond(effective)>1e8:raise ValueError('WEAK_CONDITION')
    u=np.linalg.solve(effective,-g[:d]+b@cg)
    v=np.linalg.solve(c,-g[d:]-b.T@u)
    return np.r_[u,v]


def choose_with_abstention(losses,margin):
    """Index0=baseline. This certifies only the supplied finite candidate set."""
    x=np.asarray(losses,dtype=float)
    if x.ndim!=2 or x.shape[0]<1 or x.shape[1]<2 or not np.isfinite(x).all() or np.any((x<0)|(x>1)):raise ValueError('INVALID_LOSSES')
    if not np.isfinite(margin) or margin<0:raise ValueError('INVALID_MARGIN')
    winners=[j for j in range(1,len(x)) if all(pairwise_advantage(x[j],x[k])>margin for k in range(len(x)) if k!=j)]
    return winners[0] if len(winners)==1 else None


def gyro_penalty(angle_rad,valid_contract,scale_rad):
    if not valid_contract:return None
    if angle_rad is None or not np.isfinite(angle_rad) or angle_rad<0 or not np.isfinite(scale_rad) or scale_rad<=0:raise ValueError('INVALID_GYRO')
    return float(min((angle_rad/scale_rad)**2,1.))
