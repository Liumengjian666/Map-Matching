"""Independent record checks; no optimizer, point processing or data selection."""
import itertools
import json
import unittest
from pathlib import Path

import numpy as np
from analyze_results import DOC, rows, identity


class CompletedRunTests(unittest.TestCase):
    def test_recorded_align_accounting(self):
        calls = rows(DOC/'NATIVE_ALIGN_CALLS.csv')
        self.assertEqual(len(calls), 416)
        self.assertEqual(sum(r['status']=='START' for r in calls), 208)
        self.assertEqual(sum(r['status']=='RETURNED' for r in calls), 208)
        kinds = [r['kind'] for r in calls if r['status']=='START']
        self.assertEqual(kinds.count('CONTROL_FULL'), 100)
        self.assertEqual(kinds.count('CONTROL_SPLIT'), 100)
        self.assertEqual(kinds.count('RECOVERY'), 8)

    def test_formal_and_forced_are_separate(self):
        formal = rows(DOC/'NATURAL_SHADOW_RESULTS.csv')
        forced = rows(DOC/'DIAGNOSTIC_PROBE_RESULTS.csv')
        self.assertEqual(len(formal), 100)
        self.assertEqual(sum(int(r['natural_trigger']) for r in formal), 0)
        self.assertEqual(sum(int(r['extra_calls']) for r in formal), 0)
        self.assertEqual([int(r['scan_index']) for r in forced], [20,40,60,80])
        self.assertEqual(sum(int(r['extra_calls']) for r in forced), 8)

    def test_same_pool_singleton_margin_is_missing(self):
        choices = rows(DOC/'SELECTOR_COMPARISON.csv')
        self.assertEqual(len(choices), 312)
        for group_start in range(0,len(choices),3):
            group = choices[group_start:group_start+3]
            self.assertEqual({r['pool_ids'] for r in group}, {'0'})
            self.assertTrue(all(r['min_pairwise_advantage']=='NA' for r in group))
        self.assertFalse(rows(DOC/'PAIRWISE_ADVANTAGES.csv'))

    def test_all_recovery_outputs_are_recorded_duplicates(self):
        candidates = rows(DOC/'CANDIDATE_POOL.csv')
        recoveries = [r for r in candidates if r['kind']=='RECOVERY']
        self.assertEqual(len(recoveries),8)
        self.assertTrue(all(r['duplicate_of']=='0' and r['converged']=='1' for r in recoveries))
        self.assertFalse(rows(DOC/'FAILED_NATIVE_OUTPUT.csv'))

    def test_native_output_freeze_and_copies(self):
        freeze = json.loads((DOC/'shadow_output_freeze.json').read_text())
        for r in freeze['files']:
            self.assertEqual(identity(Path(r['path'])),r)
            copy = DOC/Path(r['path']).name
            self.assertEqual(identity(copy)['sha256'],r['sha256'])
        manifest=json.loads((DOC/'REAL_INPUT_MANIFEST.json').read_text())
        self.assertEqual(identity(DOC/'WINDOW_PRE_REGISTRATION.json'),manifest['window_pre_registration'])

    def test_independent_common_weight_vertex_counterexample(self):
        # Independent enumeration of all 70 capped-simplex vertices. This is a
        # synthetic mathematical oracle, not a real candidate recovery claim.
        j=np.array([0,0,.5,.5,.5,.5,1,1]); k=np.full(8,.5)
        exact=min(float(np.sum((k-j)[list(v)]))*.25 for v in itertools.combinations(range(8),4))
        closed=float(np.mean(np.sort(k-j)[:4]))
        self.assertAlmostEqual(exact,closed,places=14)
        self.assertLess(exact,0)
        independent_robust_scores=np.mean(np.sort(k)[:4])-np.mean(np.sort(j)[:4])
        self.assertNotEqual(exact,independent_robust_scores)


if __name__=='__main__':
    unittest.main()
