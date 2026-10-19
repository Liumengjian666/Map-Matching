"""Pure SE(3) diagnostics, no optimizer, estimator or pose modification."""
import numpy as np
from scipy.spatial.transform import Rotation


def transform(values):
    values=np.asarray(values,dtype=float)
    assert values.shape==(7,) and np.isfinite(values).all()
    assert abs(np.linalg.norm(values[3:])-1)<1e-6
    T=np.eye(4);T[:3,:3]=Rotation.from_quat(values[3:]).as_matrix();T[:3,3]=values[:3]
    return T


def inverse(T):
    result=np.eye(4);result[:3,:3]=T[:3,:3].T;result[:3,3]=-T[:3,:3].T@T[:3,3]
    return result


def skew(x):
    a,b,c=x
    return np.array([[0,-c,b],[c,0,-a],[-b,a,0]],dtype=float)


def se3_log(T):
    """Principal log [rho(m),phi(rad)]; Exp(rho,phi)=(R,J_left(phi)rho)."""
    phi=Rotation.from_matrix(T[:3,:3]).as_rotvec();theta=np.linalg.norm(phi);K=skew(phi)
    if theta<1e-7:J=np.eye(3)+.5*K+(1/6)*K@K
    else:J=np.eye(3)+(1-np.cos(theta))/theta**2*K+(theta-np.sin(theta))/theta**3*K@K
    return np.r_[np.linalg.solve(J,T[:3,3]),phi]


def increment(previous,current):return inverse(previous)@current


def residual(previous_prediction,prediction,previous_raw,raw):
    dp=increment(previous_prediction,prediction);dr=increment(previous_raw,raw)
    E=inverse(dp)@dr
    return dp,dr,E,se3_log(E)


def pose_difference(A,B):
    E=inverse(A)@B
    return float(np.linalg.norm(E[:3,3])),float(np.rad2deg(np.linalg.norm(Rotation.from_matrix(E[:3,:3]).as_rotvec())))
