#!/usr/bin/env python3
"""Build only a read-only trace ELF; reuse all original algorithm libraries."""
import hashlib,json,shlex,subprocess
from pathlib import Path
A=Path(__file__).resolve().parents[1];W=A.parents[1]
D=Path('/home/jian/livox_ws/p63_r5_hdl_public_baseline_data')
C=Path('/home/jian/livox_ws/p63_r3_hdl_data/catkin_ws')
build=D/'trace_build';build.mkdir(exist_ok=False)
source=W/'research/p63_hdl_public_baseline/src/read_only_trace.cpp';obj=build/'trace.o';binary=build/'libp63_r5_trace.so'
old=C/'build/p63_hdl_candidate/CMakeFiles/p63_hdl_trace.dir'
flags=next(x.split('=',1)[1].strip() for x in (old/'flags.make').read_text().splitlines() if x.startswith('CXX_INCLUDES ='))
compile_command=['/usr/bin/c++','-std=c++14','-O3','-DNDEBUG','-fPIC',*shlex.split(flags),'-c',str(source),'-o',str(obj)]
link=shlex.split((old/'link.txt').read_text());link[link.index('-o')+1]=str(binary)
link=[str(obj) if x.endswith('src/read_only_trace.cpp.o') else x for x in link]
(A/'logs').mkdir(exist_ok=True)
with (A/'logs/build.log').open('x') as stream:
    subprocess.run(compile_command,check=True,stdout=stream,stderr=subprocess.STDOUT)
    subprocess.run(link,check=True,stdout=stream,stderr=subprocess.STDOUT)
freeze=json.loads((W/'docs/p63_r3_hdl_ndt_candidate/BUILD_AND_SOURCE_FREEZE.json').read_text())
for p,v in freeze['binaries'].items():
    assert hashlib.sha256(Path(p).read_bytes()).hexdigest()==v['sha256']
receipt={'compile_command':compile_command,'link_command':link,'build_type':'Release -O3 -DNDEBUG',
         'trace_source':str(source),'trace_source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
         'trace_binary':str(binary),'trace_binary_sha256':hashlib.sha256(binary.read_bytes()).hexdigest(),
         'all_R3_libraries_still_identical':True,'original_algorithm_libraries_rebuilt':False,
         'upstream':freeze['upstream'],'native_binaries':freeze['binaries'],
         'measurement_change':'Copy and log exact native const guess matrix; forward exactly once. Same R3 diagnostic queries.'}
(A/'BUILD.json').write_text(json.dumps(receipt,indent=2)+'\n')
print('PASS Release trace-only build; all frozen R3 algorithm/trace libraries remain untouched.')
