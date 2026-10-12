"""Exactly one bounded offline execution after capture/parity gates. No GT."""
import argparse
import csv
import json
import os
import subprocess
import time
from pathlib import Path
from prepare_capture import sha


def run(repo, probe):
    repo=repo.resolve();cache=repo/'.p23_r1_budgeted_coupled_subspace_refinement'
    gate=json.loads((cache/'capture_gate/results.json').read_text())
    if gate['SOURCE_CAPTURE']!='SOURCE_CAPTURE_PASS' or gate['BASELINE_PARITY']!='PASS':
        raise ValueError('CAPTURE_GATE_NOT_PASSED')
    freeze=json.loads((cache/'runtime_input_freeze.json').read_text())
    for path,digest in freeze['files_sha256'].items():
        if sha(path)!=digest:raise ValueError('FROZEN_INPUT_CHANGED:'+path)
    output=cache/'offline';output.mkdir(exist_ok=False)
    with (cache/'runs/capture_visual/ndt_diagnostics.csv').open() as stream:
        diag={row['frame_index']:row for row in csv.DictReader(stream)}
    cases=[]
    for row in freeze['selection']:
        frame=row['frame_index'];online=diag[frame]
        prefix=cache/'sources'/('frame_'+frame)
        metadata=json.loads(prefix.with_suffix('.source.json').read_text())
        if sha(prefix.with_suffix('.xyz'))!=metadata['xyz_sha256']:raise ValueError('CAPTURE_CHANGED')
        cases.append(' '.join(str(value) for value in (frame,metadata['scan_ns'],metadata['points'],
            metadata['width'],metadata['height'],int(metadata['is_dense']),metadata['stable_hash'],
            online['ndt_iterations'],online['ndt_has_converged'],online['ndt_fitness'],
            online['translation_limited'],online['rotation_limited'])))
    case_file=output/'cases.txt';case_file.write_text('\n'.join(cases)+'\n')
    scripts=Path(__file__).resolve().parent
    files=list((cache/'sources').iterdir())+[case_file,probe,cache/'runtime_input_freeze.json',cache/'capture_gate/results.json']
    files+=list(scripts.glob('*.hpp'))+list(scripts.glob('*.cpp'))+list(scripts.glob('*.py'))
    files+=[repo/'src/dog_prior_map_fastlio2_frontend_exp/src/coupled_ndt_shadow.cpp',
            repo/'src/dog_prior_map_fastlio2_frontend_exp/include/dog_prior_map_fastlio2_frontend_exp/observable_pcl_ndt.hpp',
            repo/'docs/p23_r1_budgeted_coupled_subspace_refinement/PROTOCOL.md']
    preregistration=dict(TASK=freeze['TASK'],GT_LOADED=False,max_frames=8,max_full_aligns=24,
        files_sha256={str(path.resolve()):sha(path) for path in files})
    (cache/'offline_input_freeze.json').write_text(json.dumps(preregistration,indent=2)+'\n')
    configuration=json.loads((cache/'input_freeze.json').read_text())
    import yaml
    map_path=yaml.safe_load(Path(configuration['capture_config']).read_text())['map']['pcd_fallback_path']
    command=[str(probe.resolve()),map_path,str(cache/'sources'),str(case_file),str(output)]
    tick=time.monotonic()
    completed=subprocess.run(command,capture_output=True,text=True,
        env=dict(os.environ,LD_PRELOAD='/lib/x86_64-linux-gnu/libusb-1.0.so.0'))
    (output/'stdout.log').write_text(completed.stdout);(output/'stderr.log').write_text(completed.stderr)
    receipt=dict(command=command,returncode=completed.returncode,wall_sec=time.monotonic()-tick,GT_LOADED=False)
    (output/'execution.json').write_text(json.dumps(receipt,indent=2)+'\n')
    # Freeze actual candidate/results before evaluation is authorized.
    paths=[path for path in output.iterdir() if path.is_file()]
    pre_gt=dict(TASK=freeze['TASK'],protocol='P23_PRE_GT_OUTPUT_FREEZE',GT_LOADED=False,
        completed=completed.returncode==0,files_sha256={str(path.resolve()):sha(path) for path in paths},
        input_freeze_sha256=sha(cache/'offline_input_freeze.json'))
    (cache/'pre_gt_freeze.json').write_text(json.dumps(pre_gt,indent=2)+'\n')
    print(json.dumps(receipt,indent=2));print(completed.stdout);print(completed.stderr)
    if completed.returncode:raise ValueError('OFFLINE_PROBE_FAILED_NO_GT_EVALUATION')


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--repo',type=Path,required=True)
    parser.add_argument('--probe',type=Path,required=True);run(**vars(parser.parse_args()))
