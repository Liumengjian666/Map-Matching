import csv,json,sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
from scipy.linalg import expm
from check_assets import HERE,sha


class ActualResultTests(unittest.TestCase):
    def test_result_scope_threshold_and_coverage(self):
        with (HERE/'interval_rotation_audit.csv').open() as f:r=list(csv.DictReader(f))
        self.assertEqual([(int(x['scan_prev']),int(x['scan_curr'])) for x in r],[(i-1,i) for i in range(5155,5166)])
        self.assertTrue(all(x['coverage_status']=='VALID' for x in r))
        self.assertTrue(all(float(x['max_imu_gap_ms'])<=3*float(x['median_imu_dt_ms']) for x in r))
        v=[float(x['angular_residual_deg']) for x in r]
        self.assertTrue(any(a>5 and b>5 for a,b in zip(v,v[1:])))
        result=json.loads((HERE/'numerical_results.json').read_text())
        self.assertEqual(result['result'],'ANGULAR_CONSISTENCY_CONTRADICTED')
        self.assertAlmostEqual(np.median(v),result['median_residual_deg'],places=12)
        self.assertFalse(result['p95_is_decision_gate'])

    def test_independent_expm_integration_of_recorded_segments(self):
        raw=json.loads((HERE/'gyro_samples.json').read_text())
        lookup={x['stamp_ns']:np.array(x['gyro_radps']) for x in raw['samples']}
        bias=np.array(json.loads((HERE/'inputs_frozen.json').read_text())['gyro_bias_I_radps'])
        details=json.loads((HERE/'integration_details.json').read_text())
        for d in details:
            R=np.eye(3);segments=d['integration']['segments']
            for s in segments:
                self.assertLessEqual(s['sample_stamp_ns'],s['start_ns'])
                self.assertGreater(s['end_ns'],s['start_ns'])
                w=(lookup[s['sample_stamp_ns']]-bias)*(s['end_ns']-s['start_ns'])*1e-9
                x,y,z=w;K=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
                R=R@expm(K)
            np.testing.assert_allclose(R,d['Delta_R_gyro'],atol=1e-13,rtol=1e-13)

    def test_frozen_assets_unchanged_and_input_pin(self):
        b=json.loads((HERE/'asset_gate_before.json').read_text());a=json.loads((HERE/'asset_gate_after.json').read_text())
        def identity(x):return [(r['path'],r['sha256'],r['size_bytes'],r['status']) for r in x['assets']]
        self.assertEqual(identity(a),identity(b));self.assertEqual(a['comparisons'],156)
        self.assertEqual(sha(HERE/'inputs_frozen.json'),'a2f3b61260e727addc935fff38cdfee587a88ba806d4a3992e094af9bb8af690')
        binding=json.loads((HERE/'gyro_payload_binding.json').read_text())
        self.assertEqual(sha(HERE/'gyro_samples.json'),binding['gyro_samples_sha256'])


if __name__=='__main__':unittest.main()
