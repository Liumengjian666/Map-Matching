"""Only compile count-extended preprocessing checker; reuse all R9 runtime ELFs."""
import hashlib,json,os,subprocess
from pathlib import Path
A=Path(__file__).resolve().parents[1];W=A.parents[1];D=Path('/home/jian/livox_ws/p63_r10_imu_full_hard_data')
B=json.loads((W/'docs/p63_r9_hdl_real_imu_prediction/BUILD.json').read_text());old=json.loads((W/'docs/p63_r8_hdl_indoor_hard_holdout/BASELINE_BUILD.json').read_text())['preflight']
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
for p,v in B['native_binaries'].items():assert sha(p)==v['sha256']
assert sha(B['trace_binary'])==B['trace_binary_sha256']
src=A/'tools/kernel_preflight.cpp';obj=D/'preflight.o';binary=D/'kernel_preflight';assert not binary.exists()
cc=old['compile_command'].copy();cc[cc.index('-c')+1]=str(src);cc[cc.index('-o')+1]=str(obj)
link=old['link_command'].copy();link[link.index('-o')+1]=str(binary);link=[str(obj) if x.endswith('.o') else x for x in link]
env=dict(os.environ,LD_LIBRARY_PATH='/usr/lib/x86_64-linux-gnu:'+os.environ.get('LD_LIBRARY_PATH',''))
with (A/'logs/build_preflight.log').open('x') as stream:
 for cmd in [cc,link]:subprocess.run(cmd,env=env,check=True,stdout=stream,stderr=subprocess.STDOUT)
B.update(preflight={'compile_command':cc,'link_command':link,'source':str(src),'source_sha256':sha(src),'binary':str(binary),'binary_sha256':sha(binary)},runtime_binaries_reused_from_R9=True)
(A/'BUILD.json').write_text(json.dumps(B,indent=2)+'\n')
print('PASS preprocessing build; no runtime library rebuild, no NDT/UKF')
