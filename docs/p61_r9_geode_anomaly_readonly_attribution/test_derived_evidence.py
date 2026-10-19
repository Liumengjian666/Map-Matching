"""Independent checks of bounded derived evidence; never executes an estimator."""
import csv,json,subprocess,unittest
import numpy as np
from scipy.linalg import logm
from asset_gate import HERE,R8,REPO
from freeze_analysis_inputs import SELECTED,PREDECESSORS
from analysis_math import transform,inverse


def rows(name):
    with (HERE/name).open() as f:return list(csv.DictReader(f))


class EvidenceTests(unittest.TestCase):
    def test_scope_and_real_geometry(self):
        a=rows('anomaly_frame_table.csv')
        self.assertEqual([int(r['scan_index']) for r in a],SELECTED)
        self.assertEqual(len(a),112)
        with (R8/'geometry.csv').open() as f:g={int(r['scan_ns']):r for r in csv.DictReader(f)}
        for r in a:
            for kind in ('initial','raw','used'):
                for key in ('nn_median_m','nn_p90_m','overlap_0p5m','overlap_2m','overlap_8m'):
                    self.assertEqual(r[kind+'_'+key],g[int(r['end_ns'])][kind+'_'+key])
                count=int(r['source_points'])*float(r[kind+'_overlap_0p5m'])
                self.assertAlmostEqual(count,round(count),places=7)

    def test_logm_independent_motion(self):
        with (R8/'frame_results.csv').open() as f:
            frames={int(r['scan_index']):r for r in csv.DictReader(f) if int(r['scan_index']) in set(SELECTED+PREDECESSORS)}
        for r in rows('candidate_increment_consistency.csv'):
            i=int(r['scan_index']);previous=frames[i-1];current=frames[i]
            def delta(key):return inverse(transform(json.loads(previous[key])))@transform(json.loads(current[key]))
            E=inverse(delta('initial_pose'))@delta('raw_ndt_pose');L=logm(E)
            actual=np.array([L[0,3],L[1,3],L[2,3],L[2,1],L[0,2],L[1,0]]).real
            keys=('residual_rho_x_m','residual_rho_y_m','residual_rho_z_m','residual_phi_x_rad','residual_phi_y_rad','residual_phi_z_rad')
            np.testing.assert_allclose(actual,[float(r[k]) for k in keys],rtol=1e-8,atol=1e-9)
            self.assertEqual(r['fused_actual_stamp_ns'],current['corrected_publish_ns'])

    def test_time_limiter_and_density(self):
        for r in rows('time_coordinate_audit.csv'):
            self.assertEqual(r['time_record_contract'],'PASS')
            self.assertLess(float(r['roundtrip_matrix_frobenius_error']),1e-9)
            self.assertEqual(r['label_audit'],'INPUT_OFFSETS_MISLABELED_AS_OUTPUT_REFERENCE')
        for r in rows('anomaly_frame_table.csv'):
            self.assertLess(float(r['translation_step_rule_error_m']),1e-10)
            self.assertLess(float(r['rotation_step_rule_error_deg']),1e-4)
        d=rows('map_support_summary.csv');self.assertEqual(len(d),336)
        for r in d:self.assertLessEqual(int(r['map_points_within_2m']),int(r['map_points_within_5m']))

    def test_history_and_assessment_contract(self):
        prefix='docs/p61_r9_geode_anomaly_readonly_attribution/'
        for name in ('REPORT.md','cause_assessment.json'):
            original=subprocess.check_output(['git','show','aea49024c5da5294b716bc17247713c4fca7b833:'+prefix+name],cwd=REPO)
            if name=='REPORT.md':self.assertTrue((HERE/name).read_bytes().startswith(original))
            else:self.assertEqual((HERE/'blocked_cause_assessment.json').read_bytes(),original)
        m=json.loads((HERE/'cause_assessment.json').read_text())
        self.assertEqual(m['final_result'],'MIXED_OR_INCONCLUSIVE')
        required={'hypothesis','status','evidence_strength','supporting_evidence','counter_evidence',
            'unresolved_questions','relevant_scan_indices','relevant_timestamps','source_files',
            'source_code_locations','conclusion_limitations'}
        for h in m['hypotheses'].values():self.assertEqual(set(h),required)


if __name__=='__main__':unittest.main()
