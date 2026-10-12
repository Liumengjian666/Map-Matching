import unittest
from run_guarded import parameter_check
import numpy as np
from mechanism import velocity_mix, before_intervention
from audit import paired


def parameters(beta):
    return dict(ndt_observation=dict(velocity_blend=beta),map=dict(pcd_fallback_path='/ssd/map.pcd'),
        output={k:'/tmp/run/'+k+'.csv' for k in ('ekf_prediction_diagnostics_csv_path','imu_deskew_csv_path',
                'ndt_diagnostics_csv_path','oosm_csv_path','oosm_fixed_lag_csv_path','runtime_csv_path')})


class GateTests(unittest.TestCase):
    def test_single_parameter_change_accepted(self):parameter_check(parameters(.6),parameters(0.))
    def test_wrong_beta_rejected(self):
        with self.assertRaises(ValueError):parameter_check(parameters(.6),parameters(.6))
    def test_other_field_change_rejected(self):
        b=parameters(0.);b['map']['pcd_fallback_path']='/disk/other.pcd'
        with self.assertRaises(ValueError):parameter_check(parameters(.6),b)

    def test_zero_mix_not_zero_velocity(self):
        v=np.array([1.,2.,3.]);w=np.array([-1.,-2.,-3.])
        np.testing.assert_array_equal(velocity_mix(v,w,0.),v)
        np.testing.assert_allclose(velocity_mix(v,w,.6),np.array([-.2,-.4,-.6]))

    def test_later_rollback_is_not_preintervention_by_old_stamp(self):
        row=dict(event_type='OOSM_ROLLBACK',state_stamp='10.10',ndt_measurement_stamp='10.11')
        self.assertFalse(before_intervention(row,10.0,10.15))
        row['ndt_measurement_stamp']='10.0';self.assertTrue(before_intervention(row,10.0,10.15))

    def test_changed_deskew_sensor_timestamp_rejected(self):
        with self.assertRaises(ValueError):paired([dict(reference_stamp='10')],[dict(reference_stamp='11')],'reference_stamp',1)

    def test_missing_or_repeated_states_rejected(self):
        with self.assertRaises(ValueError):paired([dict(state_stamp='10')],[dict(state_stamp='10')],'state_stamp',8004)
        with self.assertRaises(ValueError):paired([dict(state_stamp='10')]*2,[dict(state_stamp='10')]*2,'state_stamp',2)

    def test_changed_rollback_association_rejected(self):
        with self.assertRaises(ValueError):paired([dict(ndt_measurement_stamp='10')],[dict(ndt_measurement_stamp='11')],'ndt_measurement_stamp',1)


if __name__=='__main__':unittest.main()
