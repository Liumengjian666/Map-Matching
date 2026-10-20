import sys
import unittest
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from audit_contracts import matrices, profile_transform, boundary_policy, from_quaternion


class ContractTests(unittest.TestCase):
    def test_yaml_profile_agree(self):
        np.testing.assert_allclose(matrices()['T_I_L_YAML_xyzw'], profile_transform(), atol=1e-14)

    def test_official_chain_axes(self):
        expected = np.array([[0,-1,0],[-1,0,0],[0,0,-1]])
        for key in ['T_I_L_YAML_xyzw', 'T_I_L_URDF']:
            np.testing.assert_allclose(matrices()[key][:3,:3], expected, atol=1e-14)

    def test_urdf_yaml_difference_not_silently_merged(self):
        for a,b in [('T_I_L_URDF','T_I_L_YAML_xyzw'),('T_I_P_URDF','T_I_P_YAML_xyzw')]:
            delta = np.linalg.norm(matrices()[a][:3,3]-matrices()[b][:3,3])
            self.assertAlmostEqual(delta, .000253, places=12)

    def test_roundtrip_nontrivial_pose(self):
        t_m_i = np.eye(4)
        t_m_i[:3,:3] = Rotation.from_rotvec([.2,-.5,.1]).as_matrix()
        t_m_i[:3,3] = [2,-3,5]
        t_i_l = matrices()['T_I_L_YAML_xyzw']
        np.testing.assert_allclose(t_m_i@t_i_l@np.linalg.inv(t_i_l), t_m_i, atol=1e-14)
        self.assertFalse(np.allclose(t_m_i@t_i_l, t_i_l@t_m_i))

    def test_base_imu_attitudes_are_not_interchangeable(self):
        angle = np.linalg.norm(Rotation.from_matrix(matrices()['T_I_base_URDF'][:3,:3]).as_rotvec())
        self.assertAlmostEqual(angle, np.pi)

    def test_boundary_unchanged_and_conditional(self):
        policy = boundary_policy()
        self.assertEqual(policy['excluded_from_fusion_only'], [1])
        self.assertEqual(policy['prospective_complete_fusion_scans'], 1093)
        self.assertEqual(policy['current_time_unit_certification'], 'STILL_CONDITIONAL')

    def test_invalid_quaternion_rejected(self):
        for q in [[0,0,0,0], [float('nan'),0,0,1], [1,2,3]]:
            with self.assertRaises(ValueError):
                from_quaternion({'quaternion':q, 'translation':[0,0,0]})


if __name__=='__main__':
    unittest.main()
