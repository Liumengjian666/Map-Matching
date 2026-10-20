import unittest
from select_window import admit_windows


class AdmissionTests(unittest.TestCase):
    def rows(self,n):return [{'structural_evaluable':'1','source_parity':'PASS','causal_initial_legal':'1'} for _ in range(n)]
    def test_no_reordering_of_original_motion_ranks(self):
        a,r=admit_windows(self.rows(102),[{'raw_start_index':'3','raw_end_index':'102'},{'raw_start_index':'1','raw_end_index':'100'}])
        self.assertEqual([x['raw_start_index'] for x in a],['3','1']);self.assertFalse(r)
    def test_all100_required(self):
        rows=self.rows(101);rows[0]['structural_evaluable']='0'
        a,r=admit_windows(rows,[{'raw_start_index':'1','raw_end_index':'100'},{'raw_start_index':'2','raw_end_index':'101'}])
        self.assertEqual([x['raw_start_index'] for x in a],['2']);self.assertEqual(r[0]['structural_bad_frames'],1)
    def test_last_frame_is_not_omitted(self):
        rows=self.rows(100);rows[-1]['structural_evaluable']='0'
        a,r=admit_windows(rows,[{'raw_start_index':'1','raw_end_index':'100'}]);self.assertFalse(a);self.assertEqual(len(r),1)
    def test_no_initial_does_not_become_identity(self):
        rows=self.rows(100);rows[20]['causal_initial_legal']='0'
        a,r=admit_windows(rows,[{'raw_start_index':'1','raw_end_index':'100'}]);self.assertFalse(a)


if __name__=='__main__':unittest.main()
