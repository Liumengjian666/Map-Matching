#!/usr/bin/env python3
"""One six-call offline trial. There is no ROS master, UKF, or retry path."""
import csv,hashlib,json,os,subprocess,time
from pathlib import Path
A=Path(__file__).resolve().parents[1];W=A.parents[1]
D=Path('/home/jian/livox_ws/p63_r6_hdl_initialization_data/formal_01')
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
F=json.loads((A/'INPUT.json').read_text());B=json.loads((A/'BUILD.json').read_text())
assert [f['raw_ordinal'] for f in F['frames']]==list(range(78,84))
assert sha(B['binary'])==B['binary_sha256'] and sha(A/'tools/oracle_ndt.cpp')==B['source_sha256']
assert sha(F['map_path'])==F['map_sha256']
for f in F['frames']:assert sha(f['path'])==f['sha256']
for p,r in B['native_binaries'].items():assert sha(p)==r['sha256']
assert os.statvfs(D.parent).f_bavail*os.statvfs(D.parent).f_frsize>=20*1024**3
D.mkdir(exist_ok=False)
env=dict(os.environ,LD_LIBRARY_PATH='/usr/lib/x86_64-linux-gnu:'+os.environ.get('LD_LIBRARY_PATH',''))
env.pop('LD_PRELOAD',None)
command=[B['binary'],'--run',str(A/'INPUT.json'),str(D)]
receipt={'CODE_SHA':subprocess.check_output(['git','rev-parse','HEAD'],cwd=W,text=True).strip(),
         'argv':command,'input_sha256':sha(A/'INPUT.json'),'binary_sha256':sha(B['binary']),
         'reference_access':'Offline six oracle seeds only; never injected into online history',
         'UKF_updates':0,'ROS_replays':0,'nominal_online_align_repeats':0}
(D/'COMMAND.json').write_text(json.dumps(receipt,indent=2)+'\n')
start=time.monotonic();reason=None;returncode=None
try:
    with (D/'RUN.log').open('x') as stream:
        result=subprocess.run(command,env=env,stdout=stream,stderr=subprocess.STDOUT,timeout=120)
    returncode=result.returncode
except subprocess.TimeoutExpired:reason='EXECUTION_TIMEOUT_NO_RETRY'
finally:
    receipt.update(returncode=returncode,wall_s=time.monotonic()-start,reason=reason)
    records=[]
    if (D/'ORACLE_NATIVE.csv').exists():
        with (D/'ORACLE_NATIVE.csv').open() as stream:records=list(csv.DictReader(stream))
    receipt['completed_align_rows']=len(records)
    receipt['before_align_logged']=(D/'RUN.log').read_text().count('BEFORE_ALIGN')
    receipt['map_sha256_after']=sha(F['map_path'])
    receipt['budget_pass']=receipt['before_align_logged']<=6 and len(records)<=6
    (D/'RUN_RECEIPT.json').write_text(json.dumps(receipt,indent=2)+'\n')
    sealed={p.name:sha(p) for p in D.iterdir() if p.is_file()}
    (D/'OUTPUT_SEAL.json').write_text(json.dumps(sealed,indent=2)+'\n')
    print(json.dumps(receipt,indent=2))
assert returncode==0 and len(records)==6 and receipt['before_align_logged']==6
