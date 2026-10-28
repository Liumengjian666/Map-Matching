"""Necessary zero-align checks of the frozen ordering semantics."""
from select_candidates import rank_candidates
def c(i,s,converged=True,finite=True):
    return dict(candidate_id=i,score=s,converged=converged,pose_finite=finite)
assert [v['candidate_id'] for v in rank_candidates([c(2,3),c(1,3),c(0,3)])]==[0,1,2]
assert [v['candidate_id'] for v in rank_candidates([c(0,2),c(1,3),c(2,99,False),c(3,100,finite=False),c(4,float('nan'))])]==[1,0]
print('PASS exact ties favor smaller IDs including 0; score order and convergence/finite eligibility; align=0.')
