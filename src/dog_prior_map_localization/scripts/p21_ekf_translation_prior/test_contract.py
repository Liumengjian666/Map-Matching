import copy
import unittest
from pathlib import Path
import yaml
from run_guarded import check_parameters


class ContractTests(unittest.TestCase):
    def setUp(self):
        self.repo=Path(__file__).resolve().parents[4]
        self.params=yaml.safe_load((self.repo/'docs/p19_r1_visual_replication_tail_risk/runs/repeat_visual/rosparams.yaml').read_text())

    def test_only_new_flag_accepted(self):
        current=copy.deepcopy(self.params)
        current['lidar_update']['experimental_ekf_translation_fallback_enable']=True
        check_parameters(self.params,current,True)
        check_parameters(self.params,self.params,False)

    def test_beta_or_other_parameter_change_rejected(self):
        for section,key,value in [('ndt_observation','velocity_blend',0),('lidar_update','ndt_resolution',1.0)]:
            current=copy.deepcopy(self.params)
            current[section][key]=value
            with self.assertRaises(ValueError):check_parameters(self.params,current,False)

    def test_single_nominal_align_and_no_extra_default_subscription(self):
        node=(self.repo/'src/dog_prior_map_localization/src/dog_prior_map_ndt_node.cpp').read_text()
        self.assertEqual(node.count('ndt_.align('),1)
        self.assertIn('if (coupled_enabled_ || experimental_ekf_translation_fallback_enable_)',node)

    def test_callback_validation_before_watermark(self):
        node=(self.repo/'src/dog_prior_map_localization/src/dog_prior_map_ndt_node.cpp').read_text()
        callback=node.split('void ekfHighRateCallback(')[1].split('\nprivate:')[0]
        self.assertLess(callback.index('quaternion_norm <= 1e-9'),callback.index('latest_ekf_high_rate_stamp_ns_ = stamp_ns'))
        self.assertIn('throw std::runtime_error("P21 causal EKF stream rejected:',callback)


if __name__=='__main__':unittest.main()
