"""Step 2: replay historical additive diagnostics without new registration."""
from pathlib import Path
import csv
import sys
import numpy as np

base = Path(__file__).resolve().parent.parent/'p64_r2_q2_bounded_search_adaptation/r3_fixed_assets'
out = Path(sys.argv[1])
rows = []
for pair in ['H307_correct_wrong','H307_native_competitors','H515_boundary_rank']:
    points = np.genfromtxt(base/f'{pair}_point_deltas.csv.gz', names=True, delimiter=',')
    d = points['delta_s_a_minus_b']
    assert np.allclose(d, points['s_a']-points['s_b'], rtol=0, atol=1e-13)
    for t in [0, .01, .05, .10]:
        cut = np.abs(d) <= t
        positive = d[cut & (d > 0)].sum()
        negative = d[cut & (d < 0)].sum()
        assert abs(d.sum()-d[~cut].sum()-positive-negative)<1e-10
        rows.append(dict(pair=pair,threshold=t,n=len(d),removed=cut.sum(),
                         full_delta=d.sum(),removed_delta=positive+negative,
                         removed_positive=positive,removed_negative=negative,
                         after_delta=d[~cut].sum(),rank_flip=np.sign(d.sum())!=np.sign(d[~cut].sum())))
with (out/'neutral_deletion.csv').open('x') as stream:
    writer=csv.DictWriter(stream,fieldnames=list(rows[0]))
    writer.writeheader()
    writer.writerows(rows)
