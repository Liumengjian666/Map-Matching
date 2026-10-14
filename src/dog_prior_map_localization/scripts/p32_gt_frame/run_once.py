#!/usr/bin/env python3
"""One fixed GT-assisted diagnostic, explicitly disallowed as runtime initialization."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess


def sha(path):
    digest=hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):digest.update(block)
    return digest.hexdigest()


def main(inputs,binary,output,protocol):
    inputs,binary,output=(path.resolve() for path in (inputs,binary,output));output.mkdir(parents=True,exist_ok=False)
    code=Path(__file__).resolve().parent
    selected=json.loads((inputs/'selection_freeze.json').read_text())
    for name,digest in selected['files_SHA256'].items():
        path=Path(name)
        if path.parent!=inputs or sha(path)!=digest:raise ValueError('CONSUMED_INPUT_BINDING')
    needed={inputs/name for name in ('frames.txt','poses.f64','T_I_L.f64','T_yaml.f64')}
    needed.update(inputs/row['file'] for row in selected['selected'])
    if set(map(Path,selected['files_SHA256']))!=needed:raise ValueError('INPUT_SET')
    for name,digest in selected['source_SHA256'].items():
        if sha(Path(name))!=digest:raise ValueError('EXTRACTION_SOURCE')
    for item in selected['input_files'].values():
        if sha(Path(item['path']))!=item['SHA256']:raise ValueError('ORIGINAL_INPUT_CHANGED')
    if selected['input_files']['map']['SHA256']!='6c39e243556eddbef7a8b4bd2de25cf7c962c4ff0f52c94cfc1d994f178ae228':raise ValueError('WRONG_MAP')
    if selected['input_files']['calibration']['SHA256']!='892bf743dc0f51eabfe9d8ae71dc98663753f174910ae9cc40ee244c77a9e217':raise ValueError('WRONG_RC1')
    env=os.environ.copy();env['LD_PRELOAD']='/lib/x86_64-linux-gnu/libusb-1.0.so.0'
    env['PYTHONPYCACHEPREFIX']='/tmp/p32-r5-pycache'
    with (output/'math_test.log').open('x') as log:
        test=subprocess.call(['/usr/bin/python3',str(code/'math_test.py')],stdout=log,stderr=subprocess.STDOUT,env=env)
    if test:raise ValueError('MATH_TEST')
    freeze=dict(selection=selected,protocol_SHA256=sha(protocol),binary_SHA256=sha(binary),
        source_SHA256={str(path):sha(path) for path in sorted(code.iterdir()) if path.is_file()},
        GT_ASSISTED_DIAGNOSTIC=True,GT_RUNTIME_INITIALIZATION=False,ICP_budget=0,NDT_budget=0)
    (output/'input_freeze.json').write_text(json.dumps(freeze,indent=2)+'\n')
    command=[str(binary),str(inputs),selected['input_files']['map']['path'],str(output)]
    timed=['/usr/bin/time','-f','user_cpu_sec=%U\nsystem_cpu_sec=%S\npeak_rss_kib=%M\nwall_sec=%e','-o',str(output/'resource.txt')]+command
    with (output/'geometry.log').open('x') as log:status=subprocess.call(timed,stdout=log,stderr=subprocess.STDOUT,env=env)
    (output/'execution.json').write_text(json.dumps(dict(command=command,exit_code=status,math_test_exit_code=test,
        GT_ASSISTED_DIAGNOSTIC=True,GT_RUNTIME_INITIALIZATION=False,ICP_calls=0,NDT_calls=0),indent=2)+'\n')
    if status:raise ValueError('GEOMETRY_PROGRAM_FAILED')
    print('GT_DIAGNOSTIC_RUN_COMPLETE: no runtime initialization or localization')


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    for name in ('inputs','binary','output','protocol'):parser.add_argument('--'+name,type=Path,required=True)
    main(**vars(parser.parse_args()))
