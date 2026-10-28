#!/usr/bin/env python3
"""Compile the offline driver, linking the already frozen R5 native libraries."""
import hashlib,json,os,subprocess
from pathlib import Path
A=Path(__file__).resolve().parents[1];W=A.parents[1]
D=Path('/home/jian/livox_ws/p63_r8_indoor_hard_holdout_data')
old=json.loads((W/'docs/p63_r5_hdl_public_baseline/BUILD.json').read_text())
source=A/'tools/candidate_ndt.cpp'; obj=D/'candidate_ndt.o'; binary=D/'candidate_ndt'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
assert not binary.exists()
for path,record in old['native_binaries'].items():assert sha(path)==record['sha256']
compile_cmd=old['compile_command'].copy()
compile_cmd[compile_cmd.index('-std=c++14')]='-std=c++17'
compile_cmd[compile_cmd.index('-c')+1]=str(source);compile_cmd[compile_cmd.index('-o')+1]=str(obj)
compile_cmd+=['-msse4.2','-fopenmp']
link_cmd=[x for x in old['link_command'] if x!='-shared' and not x.startswith('-Wl,-soname,')]
link_cmd[link_cmd.index('-o')+1]=str(binary)
link_cmd=[str(obj) if x.endswith('/trace.o') else x for x in link_cmd];link_cmd+=['-fopenmp']
env=dict(os.environ,LD_LIBRARY_PATH='/usr/lib/x86_64-linux-gnu:'+os.environ.get('LD_LIBRARY_PATH',''))
with (A/'logs/build_candidate.log').open('x') as stream:
    subprocess.run(compile_cmd,stdout=stream,stderr=subprocess.STDOUT,check=True,env=env)
    subprocess.run(link_cmd,stdout=stream,stderr=subprocess.STDOUT,check=True,env=env)
receipt={'compile_command':compile_cmd,'link_command':link_cmd,'binary':str(binary),'binary_sha256':sha(binary),
         'source_sha256':sha(source),'build_type':'Release -O3 -DNDEBUG -msse4.2 -fopenmp',
         'native_binaries':old['native_binaries'],'upstream':old['upstream'],'native_core_rebuilt':False,
         'system_library_precedence':'/usr/lib/x86_64-linux-gnu first, same as R5; avoids camera SDK libusb shadowing'}
(A/'CANDIDATE_BUILD.json').write_text(json.dumps(receipt,indent=2)+'\n')
print('Release offline driver built; no native core rebuild, align=0.')
