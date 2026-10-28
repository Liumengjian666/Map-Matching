#!/usr/bin/env python3
"""Only build zero-align preflight and read-only trace; reuse native algorithms."""
import hashlib,json,os,subprocess
from pathlib import Path
A=Path(__file__).resolve().parents[1];W=A.parents[1]
D=Path('/home/jian/livox_ws/p63_r8_indoor_hard_holdout_data')
r5=json.loads((W/'docs/p63_r5_hdl_public_baseline/BUILD.json').read_text())
r7=json.loads((W/'docs/p63_r7_nonoracle_candidate_reachability/BUILD.json').read_text())
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
for p,v in r5['native_binaries'].items():assert sha(p)==v['sha256']
env=dict(os.environ,LD_LIBRARY_PATH='/usr/lib/x86_64-linux-gnu:'+os.environ.get('LD_LIBRARY_PATH',''))
records={}
for name,old,source_name in [('preflight',r7,'kernel_preflight.cpp'),('trace',r5,'read_only_trace.cpp')]:
    source=A/'tools'/source_name;obj=D/(name+'.o');binary=D/('libp63_r8_trace.so' if name=='trace' else 'kernel_preflight')
    assert not binary.exists();cc=old['compile_command'].copy();cc[cc.index('-c')+1]=str(source);cc[cc.index('-o')+1]=str(obj)
    link=old['link_command'].copy();link[link.index('-o')+1]=str(binary)
    link=[str(obj) if x.endswith('.o') else x for x in link]
    with (A/'logs'/('build_'+name+'.log')).open('x') as stream:
        subprocess.run(cc,env=env,stdout=stream,stderr=subprocess.STDOUT,check=True)
        subprocess.run(link,env=env,stdout=stream,stderr=subprocess.STDOUT,check=True)
    records[name]={'compile_command':cc,'link_command':link,'source':str(source),'source_sha256':sha(source),'binary':str(binary),'binary_sha256':sha(binary)}
records.update(trace_binary=records['trace']['binary'],trace_binary_sha256=records['trace']['binary_sha256'],
               trace_source=records['trace']['source'],trace_source_sha256=records['trace']['source_sha256'],
               native_binaries=r5['native_binaries'],upstream=r5['upstream'],core_rebuilt=False)
(A/'BASELINE_BUILD.json').write_text(json.dumps(records,indent=2)+'\n')
print('PASS Release preflight/trace only, native libraries unchanged, align=0.')
