import csv,json,sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from check_assets import HERE,REPO,sha


class FrozenInputTests(unittest.TestCase):
    def test_exact_eleven_intervals_and_integer_stamps(self):
        p=json.loads((HERE/'inputs_frozen.json').read_text())
        pairs=[(i['scan_prev'],i['scan_curr']) for i in p['intervals']]
        self.assertEqual(pairs,[(i-1,i) for i in range(5155,5166)])
        self.assertEqual(sorted(map(int,p['raw_pose_strings'])),list(range(5154,5166)))
        for i in p['intervals']:
            self.assertIsInstance(i['t_start_ns'],int);self.assertIsInstance(i['t_end_ns'],int)
            self.assertEqual(i['dt_ns'],i['t_end_ns']-i['t_start_ns']);self.assertGreater(i['dt_ns'],0)

    def test_frozen_raw_pose_bytes_and_source_identity(self):
        p=json.loads((HERE/'inputs_frozen.json').read_text())
        path=REPO/'docs/p61_r8_geode_full_return_control/frame_results.csv'
        self.assertEqual(sha(path),p['raw_pose_source_sha256'])
        with path.open() as f:r={int(x['scan_index']):x for x in csv.DictReader(f)}
        for i,value in p['raw_pose_strings'].items():self.assertEqual(value,r[int(i)]['raw_ndt_pose'])
        self.assertEqual(sha(HERE/'PRE_REGISTRATION.md'),p['pre_registration_sha256'])

    def test_full_precision_bias_and_extrinsic(self):
        p=json.loads((HERE/'inputs_frozen.json').read_text())
        r=json.loads((REPO/'docs/p61_r6_geode_causal_init_bridge/initialization.json').read_text())['initial_state']
        self.assertEqual(p['gyro_bias_I_radps'],r['gyro_bias_I']);self.assertEqual(p['T_I_L'],r['T_I_L'])
        self.assertEqual([round(x,9) for x in p['gyro_bias_I_radps']],[-.000872524,.002375748,-.010976615])


if __name__=='__main__':unittest.main()
