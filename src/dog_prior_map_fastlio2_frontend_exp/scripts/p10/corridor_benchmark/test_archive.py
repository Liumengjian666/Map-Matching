import unittest
from pathlib import Path
import tempfile
import numpy as np
import archive

def fixture(method, tx, success=True):
    row = dict(method=method, transaction_id=str(tx), ndt_effective=str(int(success)),
        ndt_status='SUCCESS' if success else 'ITERATION_LIMIT_EXHAUSTED',iterations='10' if success else '80',
        raw_score='100',fitness_m2='.01',candidate_score='100',
        pcl_converged=str(int(success)), source_count='100', source_hash='123', target_count='321',
        weak_attempted='0', weak_quality='0', triggered='0', anchor_valid='0', feedback='0',
        strong_selected='0', jet_calls='0', value_calls='0', extra_align_calls='0',
        step_m='0', step_deg='0', total_ms='10', total_cpu_ms='9', align_ms='7',common_source_ms='1',
        refinement_ms='0', peak_RSS_KiB='1024', refinement_status='NOT_RUN', strong_status='NOT_RUN')
    for prefix in ('prediction','nominal','weak','coupled','executed'):
        for r in range(3):
            for c in range(4):
                row[f'{prefix}_r{r}c{c}'] = str(float(np.eye(4)[r,c]))
    return row

class ArchiveTests(unittest.TestCase):
    def test_csv_archive_uses_LF(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'fixture.csv'
            archive.write_csv(path,[{'key':'value'}])
            self.assertEqual(path.read_bytes(),b'key\nvalue\n')

    def test_intermediate_strong_choice_is_not_accepted_feedback(self):
        row=fixture('COUPLED',52)
        row.update(strong_selected='1',strong_m='.01',strong_deg='.1',weak_score='100',coupled_score='101')
        rejected=archive.summarize([row])
        self.assertEqual(rejected['strong_candidate_selected'],1)
        self.assertEqual(rejected['strong_feedback_used'],0)
        self.assertEqual(rejected['strong_selected_but_final_rejected'],1)
        row['feedback']='1';row['weak_m']='.02'
        self.assertEqual(archive.summarize([row])['strong_feedback_used'],1)

    def test_receipt_cannot_claim_unwritten_full_sequence(self):
        data=[fixture(m,52) for m in archive.METHODS]
        receipt=dict(frames_per_arm=2726,status='FULL_ELIGIBLE_INPUT_PROCESSED',
            full_align_calls=8178,registration_align_attempts=8178)
        with self.assertRaises(ValueError): archive.validate_receipt(receipt,data,[{}])
        receipt.update(frames_per_arm=1,full_align_calls=3,registration_align_attempts=3)
        with self.assertRaises(ValueError): archive.validate_receipt(receipt,data,[{}])
        receipt['status']='FIRST_NDT_STRICT_STARTUP_FAILED'
        self.assertEqual(archive.validate_receipt(receipt,data,[{}])[0],1)
        with self.assertRaises(ValueError): archive.validate_receipt(receipt,data,[{},{}])

    def test_convergence_transitions_not_position_recoveries(self):
        data = [fixture('NOMINAL', 52+i, success) for i,success in enumerate((True,False,False,True,False))]
        summary = archive.summarize(data)
        self.assertEqual(summary['effective'],2)
        self.assertEqual(summary['failure_episodes'],2)
        self.assertEqual(summary['failure_to_success_transitions'],1)
        self.assertEqual(summary['longest_failure_streak'],2)
        self.assertEqual(summary['true_localization_recoveries'],'NOT_AVAILABLE')

    def test_audit_rejects_source_budget_future_and_nonfinite(self):
        def inputs():
            return [fixture(m,52) for m in archive.METHODS], [dict(IMU_max_consumed_ns='20',
                scan_end_ns='21', IMU_gap_ns='5', endpoint_hold_ns='1')], dict(GT_LOADED=False,IKFOM_RUN=False,source_hashes={})
        data,ledger,freeze=inputs()
        self.assertEqual(archive.audit(data,ledger,freeze)['status'],'PASS')
        for field,value in [('source_hash','999'),('jet_calls','1'),('executed_r0c0','nan')]:
            data,ledger,freeze=inputs();data[0][field]=value
            with self.assertRaises(ValueError): archive.audit(data,ledger,freeze)
        data,ledger,freeze=inputs();ledger[0]['IMU_max_consumed_ns']='22'
        with self.assertRaises(ValueError): archive.audit(data,ledger,freeze)

if __name__=='__main__': unittest.main()
