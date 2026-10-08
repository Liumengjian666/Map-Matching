import unittest
import numpy as np
from preintegration import integrate, motion_system, profile_diagnostics, exp
from prepare_bootstrap import verify_input

class PreintegrationTests(unittest.TestCase):
    def test_consumed_input_cannot_be_omitted_from_manifest(self):
        with self.assertRaisesRegex(RuntimeError,"omitted"):
            verify_input({"input_files":{}})
    def test_constant_motion_with_real_bias(self):
        stamps=np.arange(0, 1000000001, 5000000, dtype=np.int64)
        measurements=np.tile([1.,2.,3.,.01,-.02,.03],(len(stamps),1))
        R,v,p,Jv,Jp=integrate(stamps,measurements,0,1000000000,np.array([.01,-.02,.03]))
        np.testing.assert_allclose(R,np.eye(3),atol=1e-14)
        np.testing.assert_allclose(v,[1,2,3],atol=1e-13)
        np.testing.assert_allclose(p,[.5,1,1.5],atol=1e-13)
        np.testing.assert_allclose(Jv,np.eye(3),atol=1e-13)
        np.testing.assert_allclose(Jp,.5*np.eye(3),atol=1e-13)

    def test_no_backfill_and_bounded_hold(self):
        stamps=np.array([1000000,6000000,11000000],dtype=np.int64)
        measurements=np.zeros((3,6))
        with self.assertRaises(ValueError): integrate(stamps,measurements,0,11000000,np.zeros(3))
        with self.assertRaises(ValueError): integrate(stamps,measurements,1000000,22000000,np.zeros(3))
        integrate(stamps,measurements,1000000,16000000,np.zeros(3))

    def test_future_sample_cannot_change_endpoint(self):
        stamps=np.array([1000000,6000000,11000000],dtype=np.int64)
        measurements=np.zeros((3,6)); altered=measurements.copy(); altered[-1]=1000.
        first=integrate(stamps,measurements,1000000,8000000,np.zeros(3))
        second=integrate(stamps,altered,1000000,8000000,np.zeros(3))
        for a,b in zip(first,second):np.testing.assert_array_equal(a,b)

    def test_begin_bracket_cannot_hide_missing_imu(self):
        stamps=np.array([-100000000,5000000,10000000],dtype=np.int64)
        with self.assertRaisesRegex(ValueError,"bracket"):
            integrate(stamps,np.zeros((3,6)),0,10000000,np.zeros(3))

    def test_accel_bias_integrals_and_motion_equation_signs(self):
        stamps=np.arange(0,1000000001,5000000,dtype=np.int64)
        samples=np.tile([1.,2.,9.,.1,.2,.3],(len(stamps),1))
        ba=np.array([.3,-.2,.1]); bg=np.array([.01,.02,.03])
        corrected=samples.copy(); corrected[:,:3]-=ba
        R,v,p,Jv,Jp=integrate(stamps,samples,0,1000000000,bg)
        _,vc,pc,_,_=integrate(stamps,corrected,0,1000000000,bg)
        np.testing.assert_allclose(v-Jv@ba,vc,atol=1e-12)
        np.testing.assert_allclose(p-Jp@ba,pc,atol=1e-12)
        poses=np.tile(np.eye(4),(2,1,1)); poses[0,:3,:3]=exp(np.array([.4,.1,-.3]))
        gravity=np.array([.2,.3,-np.sqrt(9.809**2-.2**2-.3**2)])
        vi=np.array([1.,-.2,.3]); R0=poses[0,:3,:3]
        poses[1,:3,:3]=R0@R
        poses[1,:3,3]=vi+.5*gravity+R0@pc
        vj=vi+gravity+R0@vc
        _,_,A,y=motion_system(poses,np.array([0,1000000000]),[(R,v,p,Jv,Jp)],[[1.,1.]])
        np.testing.assert_allclose(A@np.concatenate((vi,vj,gravity,ba)),y,atol=1e-12)

    def test_constant_orientation_cannot_separate_g_and_ba(self):
        times=np.arange(6,dtype=np.int64)*500000000
        poses=np.tile(np.eye(4),(6,1,1)); poses[:,0,3]=times*1e-9
        integral=(np.eye(3),np.array([0,0,9.809])*.5,np.array([0,0,9.809])*.125,
                  np.eye(3)*.5,np.eye(3)*.125)
        sigmas=np.tile([.05,.2],(5,1))
        A,_,_,_=motion_system(poses,times,[integral]*5,sigmas)
        d=profile_diagnostics(A,6,np.array([0,0,-9.809]),[.856,.856,.5,.5,.5],
                              np.zeros(5),np.ones(5)*.5,sigmas)
        self.assertEqual(d['profile_rank'],3)
        self.assertLess(d['sigma_min_lower_bound'],1e-10)

    def test_perturbation_cannot_certify_zero_excitation(self):
        times=np.arange(6,dtype=np.int64)*500000000
        poses=np.tile(np.eye(4),(6,1,1))
        for i in range(6): poses[i,:3,:3]=exp(np.array([.1*i,.01*i*i,.03*i]))
        integral=(np.eye(3),np.zeros(3),np.zeros(3),np.eye(3)*.5,np.eye(3)*.125)
        sigmas=np.tile([.05,.2],(5,1))
        A,_,_,_=motion_system(poses,times,[integral]*5,sigmas)
        d=profile_diagnostics(A,6,np.array([0,0,-9.809]),[.856,.856,.5,.5,.5],
                              np.ones(5)*.5,np.ones(5)*.5,sigmas)
        self.assertEqual(d['sigma_min_lower_bound'],0)
        self.assertIsNone(d['robust_condition'])

if __name__=='__main__': unittest.main()
