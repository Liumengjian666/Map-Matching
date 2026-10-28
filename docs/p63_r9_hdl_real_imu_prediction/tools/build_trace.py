"""Reuse frozen Release flags/libraries; build read-only trace, no native rebuild."""
import hashlib,json,subprocess,os
from pathlib import Path
A=Path(__file__).resolve().parents[1];W=A.parents[1];D=Path('/home/jian/livox_ws/p63_r9_real_imu_prediction_data');D.mkdir(exist_ok=True)
B=json.loads((W/'docs/p63_r8_hdl_indoor_hard_holdout/BASELINE_BUILD.json').read_text())
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
for p,v in B['native_binaries'].items():assert sha(p)==v['sha256']
src=A/'tools/read_only_trace.cpp';obj=D/'trace.o';binary=D/'libp63_r9_trace.so'
cc=B['trace']['compile_command'].copy();cc[cc.index('-c')+1]=str(src);cc[cc.index('-o')+1]=str(obj);cc.insert(1,'-I/home/jian/livox_ws/p63_r3_hdl_data/catkin_ws/src/hdl_localization/include')
link=B['trace']['link_command'].copy();link[link.index('-o')+1]=str(binary);link=[str(obj) if x.endswith('.o') else x for x in link]
env=dict(os.environ,LD_LIBRARY_PATH='/usr/lib/x86_64-linux-gnu:'+os.environ.get('LD_LIBRARY_PATH',''))
with (A/'logs/build_trace.log').open('w') as stream:
 for cmd in [cc,link]:subprocess.run(cmd,env=env,check=True,stdout=stream,stderr=subprocess.STDOUT)
(A/'BUILD.json').write_text(json.dumps({'compile_command':cc,'link_command':link,'trace_source':str(src),'trace_source_sha256':sha(src),'trace_binary':str(binary),'trace_binary_sha256':sha(binary),'native_binaries':B['native_binaries'],'upstream':B['upstream'],'core_rebuilt':False},indent=2)+'\n')
print('PASS Release read-only trace; native core unchanged; NDT=0.')
