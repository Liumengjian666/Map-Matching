#!/usr/bin/env python3
"""Check public timestamps/schema only, with no reference CSV or error fitting."""
from pathlib import Path
from decimal import Decimal
import hashlib,json,re,math
S=Path('/home/jian/livox_ws/p62_r26_published_baseline_sources')
D=Path(__file__).resolve().parents[1]
P=Path('/home/jian/livox_ws/p59_r3_newer_college_official_internal_recovery_data/01_short_experiment/raw_format/ouster_scan')
scans=[]
for p in P.glob('*.pcd'):
 m=re.fullmatch(r'cloud_(\d+)_(\d{9})\.pcd',p.name)
 if m:scans.append(int(m[1])*10**9+int(m[2]))
scans.sort();assert len(scans)==15301
out=[]
for method in ['steamlio','singer_inertial']:
 path=S/(method+'_01_short_trajectory.txt');rows=[line.split() for line in path.read_text().splitlines() if line and not line.startswith('#')]
 assert all(len(r)==8 for r in rows)
 ns=[int(Decimal(r[0])*Decimal(10**9)) for r in rows]
 assert all(math.isfinite(float(v)) for r in rows for v in r)
 qerr=max(abs(sum(float(v)**2 for v in r[4:])-1) for r in rows)
 out.append({'method':method,'rows':len(rows),'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'first_stamp_ns':ns[0],'last_stamp_ns':ns[-1],'all_timestamps_equal_local_pcd_order':ns==scans,'max_quaternion_squared_norm_error':qerr,'first_translation_m':[float(v) for v in rows[0][1:4]],'first_xyzw':[float(v) for v in rows[0][4:]],'coordinate_contract':'TUM schema and times verified; map-from-trajectory transform and sensor-frame binding to our conditional Base not certified','re_evaluation':'NOT_PERFORMED: no coordinate fitting, no matched C3 initial condition, no method run'})
(D/'PUBLIC_TRAJECTORY_ASSESSMENT.json').write_text(json.dumps(out,indent=2)+'\n')
print(json.dumps(out,indent=2))
