"""Zero-NDT tests: direction, static force, rigid lever arm, nonmutation/causality."""
import numpy as np
from imu_geometry import R_L_I, P_L_I, convert
assert np.allclose(R_L_I.T @ R_L_I, np.eye(3), atol=1e-14)
assert abs(np.linalg.det(R_L_I)-1)<1e-14
f=np.array([1.,2.,3.]);w=np.zeros(3);before=f.copy()
out,wl,lever=convert(f,w,w,.001)
assert np.array_equal(f,before) and np.allclose(out,R_L_I@f) and np.array_equal(lever,np.zeros(3))
# Constant spin: known centripetal acceleration at I, remove it at L.
wl=np.array([0.,0.,2.]);fl=np.array([0.,0.,9.80665]);w=R_L_I.T@wl
fi=R_L_I.T@(fl+np.cross(wl,np.cross(wl,P_L_I)))
out,actual,_=convert(fi,w,w,.01)
assert np.allclose(out,fl) and np.allclose(actual,wl)
# Constant angular acceleration: backward difference exact for this test.
alpha=np.array([.3,-.2,.1]);prev=R_L_I.T@(wl-alpha*.01)
fi=R_L_I.T@(fl+np.cross(alpha,P_L_I)+np.cross(wl,np.cross(wl,P_L_I)))
out,_,_=convert(fi,w,prev,.01)
assert np.allclose(out,fl,atol=1e-12)
for dt in [0,-1,float('nan')]:
    try:convert(fi,w,prev,dt)
    except ValueError:pass
    else:raise AssertionError('Invalid time accepted')
print('PASS: rotation direction, no sign inversion, centrifugal/tangential terms, no input mutation, invalid dt rejected. NDT=0 UKF=0.')
