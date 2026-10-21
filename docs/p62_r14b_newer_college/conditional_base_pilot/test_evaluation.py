import unittest
import json
import tempfile
from pathlib import Path
from unittest.mock import patch
import numpy as np
from scipy.spatial.transform import Rotation
from evaluate import errors, statistics, matrix, verify_native_tables, reference_lidar_pose
import evaluate


def fixture():
    prereg={'frames':[{'stamp_ns':100+i} for i in range(21)]}
    identity={f'{a}{b}':str(float(a==b)) for a in range(4) for b in range(4)}
    native=[];final=[];calls=[];events=[];raw=[]
    for i in range(1,21):
        r={'frame':str(i),'stamp_ns':str(100+i),'proposed_observation_valid':'1','status':'ACCEPT_NOMINAL'}
        for p in ['prediction_','full_','split_','accepted_']:r.update({p+k:v for k,v in identity.items()})
        final.append(r);events.append({'frame':str(i),'stamp_ns':str(100+i),'extra_calls':'0','permitted':'0'})
        for method,id in [('CONTROL_FULL',-1),('CONTROL_SPLIT',0)]:
            base={'frame':str(i),'method':method,'candidate_id':str(id),'grid_id':'-1','stamp_ns':str(100+i)}
            call=str(len(raw)+1);calls.extend([dict(base,call=call,status='START'),dict(base,call=call,status='RETURNED')])
            raw.append(dict(base,call=call));n=dict(base,eligible='1')
            for p in ['final_','seed_']:n.update({p+k:v for k,v in identity.items()})
            native.append(n)
    return prereg,native,final,calls,events,raw


class EvaluationTests(unittest.TestCase):
    def test_reference_relative_error(self):
        gt=np.eye(4);p=np.eye(4);p[:3,3]=[3,4,0]
        p[:3,:3]=Rotation.from_euler('z',90,degrees=True).as_matrix()
        t,r=errors(p,gt);self.assertAlmostEqual(t,5);self.assertAlmostEqual(r,90)

    def test_zero_and_pi_rotation(self):
        gt=np.eye(4)
        self.assertEqual(errors(gt,gt),(0,0))
        p=gt.copy();p[:3,:3]=Rotation.from_euler('x',180,degrees=True).as_matrix()
        self.assertAlmostEqual(errors(p,gt)[1],180)

    def test_rejected_not_zero_statistic(self):
        s=statistics([]);self.assertEqual(s['count'],0);self.assertIsNone(s['rmse'])

    def test_quantile_declared_linear(self):
        self.assertAlmostEqual(statistics([0,10])['p95'],9.5)

    def test_invalid_pose_rejected(self):
        p=np.eye(4);p[0,0]=-1
        row={f'p{i}{j}':str(p[i,j]) for i in range(4) for j in range(4)}
        with self.assertRaises(ValueError):matrix(row,'p')

    def test_complete_native_admission_without_reference_reader(self):
        self.assertEqual(verify_native_tables(*fixture())['CONTROL_FULL'],20)

    def test_missing_native_rejected_before_reference(self):
        values=fixture();values[1].pop()
        with self.assertRaises(RuntimeError):verify_native_tables(*values)

    def test_duplicate_call_id_rejected_before_reference(self):
        values=fixture();values[3][2]['call']='1';values[3][3]['call']='1'
        with self.assertRaises(RuntimeError):verify_native_tables(*values)

    def test_unknown_method_rejected_before_reference(self):
        values=fixture();values[3][0]['method']='OTHER'
        with self.assertRaises(RuntimeError):verify_native_tables(*values)

    def test_negative_extra_rejected_before_reference(self):
        values=fixture();values[4][0]['extra_calls']='-1'
        with self.assertRaises(RuntimeError):verify_native_tables(*values)

    def test_bad_rejected_observation_not_disguised_as_valid_pose(self):
        values=fixture();values[2][0]['proposed_observation_valid']='0';values[2][0]['status']='NO_ACCEPTABLE_MEASUREMENT'
        with self.assertRaises(RuntimeError):verify_native_tables(*values)

    def test_noncommuting_reference_extrinsic_chain(self):
        q=Rotation.from_euler('z',90,degrees=True).as_quat()
        t_bl=np.eye(4);t_bl[:3,3]=[.1,0,.2];t_bl[:3,:3]=Rotation.from_euler('x',30,degrees=True).as_matrix()
        t=reference_lidar_pose([1,2,3,*q],t_bl)
        np.testing.assert_allclose(t[:3,3],[1,2.1,3.2],atol=1e-12)
        self.assertGreater(np.linalg.norm(t[:3,:3]-t_bl[:3,:3]@Rotation.from_quat(q).as_matrix()),.1)

    def test_valid_but_different_final_matrix_rejected(self):
        values=fixture();values[2][0]['full_03']='1'
        with self.assertRaisesRegex(RuntimeError,'nominal matrix mismatch'):verify_native_tables(*values)

    def test_accepted_matrix_must_be_eligible_candidate(self):
        values=fixture();values[1][1]['eligible']='0'
        with self.assertRaisesRegex(RuntimeError,'eligible pool candidate'):verify_native_tables(*values)

    def test_mismatched_execution_binding_stops_main_before_reference(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder)
            for name,value in [('TEST_FRAME_PREREGISTRATION.json',{}),
                ('EXECUTION_FREEZE.json',{'preregistration_sha256':'wrong','binary_sha256':'binary'}),
                ('FORMAL_STARTED.json',{'preregistration_sha256':'wrong','binary_sha256':'binary'})]:
                (root/name).write_text(json.dumps(value))
            with patch.object(evaluate,'HERE',root):
                with self.assertRaisesRegex(RuntimeError,'binding mismatch'):evaluate.main()
            self.assertFalse((root/'EVALUATION_ADMISSION.json').exists())

    def test_incomplete_seal_stops_main_before_reference(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);prereg=root/'TEST_FRAME_PREREGISTRATION.json'
            prereg.write_text(json.dumps({'data_root':folder}))
            identity={'preregistration_sha256':evaluate.sha(prereg),'binary_sha256':'binary'}
            for name,value in [('EXECUTION_FREEZE.json',identity),('FORMAL_STARTED.json',identity),
                ('FORMAL_FINISHED.json',{'returncode':0,'stop_reason':None}),
                ('NATIVE_OUTPUT_FREEZE.json',{'files':[]})]:
                (root/name).write_text(json.dumps(value))
            with patch.object(evaluate,'HERE',root):
                with self.assertRaisesRegex(RuntimeError,'seal coverage incomplete'):evaluate.main()
            self.assertFalse((root/'EVALUATION_ADMISSION.json').exists())


if __name__=='__main__':unittest.main(verbosity=2)
